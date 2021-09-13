#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <sys/wait.h>
#include <math.h>
#include <semaphore.h>
#include <regex.h>

#define AW_AUTO_GET_DEV

static char *fops_cmd[] = {
	"start_cali",
	"re_cali",
	"f0_cali",
	"q_cali",
	"fast_start_cali",
	"fast_re_cali",
	"store_fixed_re",
};

struct i2c_info {
	unsigned int bus;
	unsigned int addr;
};

struct i2c_dev {
	unsigned char dev_num;
	struct i2c_info *i2c_info;
};

struct aw_re_f0 {
	unsigned int fixed_re;
	unsigned int f0;
	float q;
};

static int awinic_found_err(char *buf)
{
	char *err_p = NULL;

	err_p = strstr(buf, "ERR");
	if (err_p == NULL)
		return 0;
	else
		return -1;
}

static int awinic_call_exe(char *cmd)
{
	int ret;
	int fd[2];
	pid_t pid;
	char *ret_str = NULL;
	char buf[4096] = { 0 };

	if (pipe(fd) < 0) {
		perror("pipe error");
		exit(1);
	}

	pid = fork();
	if (pid == 0) {
		close(fd[0]);
		if (dup2(fd[1], STDOUT_FILENO) != STDOUT_FILENO) {
			perror("dup2 error");
		}
		system(cmd);
		close(fd[1]);
		exit(0);
	} else if (pid > 0) {
		close(fd[1]);
		waitpid(pid, NULL, 0);
		ret = read(fd[0], buf, sizeof(buf));
		if (ret < 0) {
			printf("read failed\n");
			close(fd[0]);
			return ret;
		}
		close(fd[0]);
		//printf("num:%d\n info:\n%s", ret, buf);
		if (awinic_found_err(buf)) {
			printf("%s execution error\n", cmd);
			return -1;
		}
	} else {
		printf("fork failed\n");
		return -1;
	}

	return 0;
}


static void awinic_connect_cmd(char *cmd, char *add_cmd)
{
	while (*cmd != '\0')
		cmd++;

	while (*add_cmd != '\0')
		*cmd++ = *add_cmd++;

	add_cmd--;
	if (*add_cmd != ' ')
		*cmd++ = ' ';
}

static int awinic_assemble_cmd(char *cmd,
	char *exe_name, char *cmd_type, char *dev_name)
{

	if (cmd_type == NULL) {
		printf("cmd_type not set\n");
		return -1;
	}
	if (exe_name == NULL) {
		printf("exe_name not set ,use default aw881xx_cali\n");
		exe_name = "aw881xx_cali";
	}
	if (dev_name == NULL) {
		printf("dev_name not set ,use default aw881xx_smartpa\n");
		exe_name = "aw881xx_smartpa";
	}

	awinic_connect_cmd(cmd, exe_name);
	awinic_connect_cmd(cmd, cmd_type);
	awinic_connect_cmd(cmd, dev_name);

	return 0;
}

static void awinic_add_i2c_to_cmd(char *cmd,
			int i2c_bus, int i2c_addr)
{
	char buf[6] = {0};

	snprintf(buf, 6, "0x%02d", i2c_bus);
	awinic_connect_cmd(cmd, buf);

	snprintf(buf, 6, "0x%02d", i2c_addr);
	awinic_connect_cmd(cmd, buf);
}

static int awinic_get_re(struct i2c_info *i2c_info,
				char *dev_name, unsigned int *re)
{
	int fd_re;
	int ret;
	char buf[32] = { 0 };
	char node_path[128] = {"/sys/bus/i2c/drivers"};

	snprintf(node_path, sizeof(node_path), "%s/%s/%d-00%d/re",
		node_path, dev_name, i2c_info->bus, i2c_info->addr);

	fd_re = open(node_path, O_RDWR);
	if (fd_re < 0) {
		printf("%s: open node re path:%s error\n",
			__func__, node_path);
		return ret;
	}

	ret = read(fd_re, buf, sizeof(buf));
	if (ret < 0) {
		printf("%s: read re node error\n", __func__);
		return ret;
	}

	sscanf(buf, "re=%dmohm", re);

	return 0;
}

static int awinic_get_f0(struct i2c_info *i2c_info,
			char *dev_name, unsigned int *f0)
{
	int fd_f0;
	int ret;
	char buf[32] = { 0 };
	char node_path[128] = {"/sys/bus/i2c/drivers"};

	snprintf(node_path, sizeof(node_path), "%s/%s/%d-00%d/f0",
		node_path, dev_name, i2c_info->bus, i2c_info->addr);

	fd_f0 = open(node_path, O_RDWR);
	if (fd_f0 < 0) {
		printf("%s: open node f0 path:%s error\n",
			__func__, node_path);
		return ret;
	}

	ret = read(fd_f0, buf, sizeof(buf));
	if (ret < 0) {
		printf("%s: read f0 node error\n", __func__);
		return ret;
	}

	sscanf(buf, "f0=%dHz", f0);

	return 0;

}

static int awinic_get_q(struct i2c_info *i2c_info,
			char *dev_name, float *q)
{
	int fd_q;
	int ret;
	int fixed_q = 0;
	char buf[32] = { 0 };
	char node_path[128] = {"/sys/bus/i2c/drivers"};

	snprintf(node_path, sizeof(node_path), "%s/%s/%d-00%d/q",
		node_path, dev_name, i2c_info->bus, i2c_info->addr);

	fd_q = open(node_path, O_RDWR);
	if (fd_q < 0) {
		printf("%s: open node q path:%s error\n",
			__func__, node_path);
		return ret;
	}

	ret = read(fd_q, buf, sizeof(buf));
	if (ret < 0) {
		printf("%s: read q node error\n", __func__);
		return ret;
	}

	sscanf(buf, "q=%d", &fixed_q);
	*q = (float)fixed_q / 1000.0;

	return 0;
}

static int awinic_start_cali(char *cmd, char *dev_name,
		struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0)
{
	int ret;
	int i;

	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_re(&i2c_dev.i2c_info[i],
			dev_name, &aw_re_f0[i].fixed_re);
		if (ret < 0)
			return ret;

		ret = awinic_get_f0(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].f0);
		if (ret < 0)
			return ret;

		ret = awinic_get_q(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].q);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int awinic_re_cali(char *cmd, char *dev_name,
		struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0)
{
	int ret;
	int i;

	printf("%s\n", cmd);
	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_re(&i2c_dev.i2c_info[i],
			dev_name, &aw_re_f0[i].fixed_re);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int awinic_f0_cali(char *cmd, char *dev_name,
	struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0)
{
	int ret;
	int i;

	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_f0(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].f0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int awinic_q_cali(char *cmd, char *dev_name,
	struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0)
{
	int ret;
	int i;

	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_q(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].q);
		if (ret < 0)
			return ret;
	}

	return 0;
}


static int awinic_store_fixed_re(char *cmd, int fixed_re)
{
	char fixed_re_buf[16] = {0};
	int ret;

	if (!fixed_re) {
		printf("re not set, re value:%d\n", fixed_re);
		return -1;
	}

	snprintf(fixed_re_buf, 16, "%d", fixed_re);
	awinic_connect_cmd(cmd, fixed_re_buf);
	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	return 0;
}

static int awinic_fast_start_cali(char *cmd, char *dev_name,
	struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0, unsigned int time)
{
	char time_buf[16] = {0};
	int ret;
	int i;

	snprintf(time_buf, 16, "%d", time);
	awinic_connect_cmd(cmd, time_buf);
	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_re(&i2c_dev.i2c_info[i],
			dev_name, &aw_re_f0[i].fixed_re);
		if (ret < 0)
			return ret;

		ret = awinic_get_f0(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].f0);
		if (ret < 0)
			return ret;

		ret = awinic_get_q(&i2c_dev.i2c_info[i],
				dev_name, &aw_re_f0[i].q);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int awinic_fast_re_cali(char *cmd, char *dev_name,
	struct i2c_dev i2c_dev, struct aw_re_f0 *aw_re_f0, unsigned int time)
{
	int ret;
	int i;
	char time_buf[16] = {0};

	snprintf(time_buf, 16, "%d", time);
	awinic_connect_cmd(cmd, time_buf);
	printf("%s\n", cmd);

	ret = awinic_call_exe(cmd);
	if (ret < 0) {
		printf("%s execution error\n", cmd);
		return ret;
	}

	for (i = 0; i < i2c_dev.dev_num; i++) {
		ret = awinic_get_re(&i2c_dev.i2c_info[i],
			dev_name, &aw_re_f0[i].fixed_re);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int awinic_get_dev_accord_regular_exp(char *data_buf,
						struct i2c_dev *i2c_dev)
{
	char *pattern = "\\s*[0-9a-fA-F]-00[0-9a-fA-F]{2}\\s*";
	regmatch_t pmatch;
	regex_t reg;
	char *p_data =NULL;
	char*match =NULL;
	char errbuf[1024] = { 0 };
	int ret;
	char i2c_buf[10] = { 0 };

	if(regcomp(&reg,pattern, REG_EXTENDED) < 0) {
		regerror(ret, &reg, errbuf, sizeof(errbuf));
		printf("%s:regcomp error: %s\n", __func__, errbuf);
		return -1;
	}

	p_data = data_buf;
	while (1) {
		ret = regexec(&reg, p_data, 1, &pmatch, REG_NOTBOL);
		if (ret == REG_NOMATCH) {
			regfree(&reg);
			break;
		} else if (ret) {
			regerror(ret, &reg, errbuf, sizeof(errbuf));
			printf("%s:regexec error: %s\n", __func__, errbuf);
			regfree(&reg);
			return -1;
		}

		i2c_dev->dev_num++;
		i2c_dev->i2c_info = (struct i2c_info *)realloc(i2c_dev->i2c_info,
					i2c_dev->dev_num * sizeof(struct i2c_info));
		if (!i2c_dev->i2c_info) {
			printf("%s: realloc i2c_dev->i2c_info error\n", __func__);
			return -1;
		}
		memset(&i2c_dev->i2c_info[i2c_dev->dev_num - 1],
			0, sizeof(struct i2c_info));

		match = p_data + pmatch.rm_so;

		memcpy(i2c_buf, match, pmatch.rm_eo-pmatch.rm_so);
		sscanf(i2c_buf, "%d-00%d", &i2c_dev->i2c_info[i2c_dev->dev_num - 1].bus,
				&i2c_dev->i2c_info[i2c_dev->dev_num - 1].addr);

		p_data += pmatch.rm_eo;
	}

	if (!i2c_dev->dev_num) {
		printf("%s: not found i2c node\n", __func__);
		return -1;
	}

	return 0;
}


static int awinic_auto_get_dev(char *dev_name,
				struct i2c_dev *i2c_dev)
{
	int ret, fd[2];
	char cmd_buf[64] = "ls /sys/bus/i2c/drivers/";
	char data_buf[4096] = {0};
	pid_t pid;

	strcat(cmd_buf, dev_name);

	if (pipe(fd) < 0) {
		printf("%s: pipe error\n", __func__);
		exit(1);
	}

	pid = fork();
	if (pid == 0) {
		close(fd[0]);
		if (dup2(fd[1], STDOUT_FILENO) != STDOUT_FILENO)
			printf("%s: dup2 error\n", __func__);

		system(cmd_buf);
		close(fd[1]);
		exit(0);
	} else if (pid > 0) {
		close(fd[1]);
		waitpid(pid, NULL, 0);
		ret = read(fd[0], data_buf, sizeof(data_buf));
		if (ret < 0) {
			close(fd[0]);
			return ret;
		}
		close(fd[0]);

		ret = awinic_get_dev_accord_regular_exp(data_buf, i2c_dev);
	} else {
		printf("%s: fork failed\n", __func__);
		return -1;
	}

	return ret;
}


int main(int argc, char *argv[])
{
	int ret;
	char cmd[100] = { 0 };
	float f0[8] = { 0 };
	int i;
	struct aw_re_f0 *aw_re_f0 = NULL;
	struct i2c_dev i2c_dev;

	/*must write*/
	char *exe_name = "aw881xx_cali";
	char *cmd_type = fops_cmd[0];
	char dev_name[30] = {"aw881xx_smartpa"};

	/*store_fixed_re must set*/
	int store_fixed_re = 8000;

	/*fast_start_cali and fast_re_cali must set*/
	int time = 10000;

	memset(&i2c_dev, 0, sizeof(struct i2c_dev));

#ifdef AW_AUTO_GET_DEV
	/*Automatic search for I2C device*/
	awinic_auto_get_dev(dev_name, &i2c_dev);
#else
	/*define i2c device*/
	i2c_dev.dev_num = 1;
	i2c_dev.i2c_info = calloc(i2c_dev.dev_num, sizeof(struct i2c_info));
	i2c_dev.i2c_info[0].addr = 34;
	i2c_dev.i2c_info[0].bus = 6;
/*	i2c_dev.i2c_info[1].addr = 35;
	i2c_dev.i2c_info[1].bus = 6;*/
#endif

	/*init aw_re_f0*/
	aw_re_f0 = calloc(i2c_dev.dev_num, sizeof(struct aw_re_f0));

	ret = awinic_assemble_cmd(cmd, exe_name, cmd_type, dev_name);
	if (ret < 0)
		return -1;

	if (i2c_dev.dev_num == 1)
		awinic_add_i2c_to_cmd(cmd, i2c_dev.i2c_info[0].bus,
					i2c_dev.i2c_info[0].addr);


	if (strcmp(*(fops_cmd + 0), cmd_type) == 0) {
		ret = awinic_start_cali(cmd, dev_name, i2c_dev, aw_re_f0);
		if (ret < 0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]fixed_re=%dmohm, f0=%dHz ,q=%f\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].fixed_re, aw_re_f0[i].f0, aw_re_f0[i].q);
		}
	} else if (strcmp(*(fops_cmd + 1), cmd_type) == 0) {
		ret = awinic_re_cali(cmd, dev_name, i2c_dev, aw_re_f0);
		if (ret < 0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]fixed_re=%dmohm\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].fixed_re);
		}
	} else if (strcmp(*(fops_cmd + 2), cmd_type) == 0) {
		ret = awinic_f0_cali(cmd, dev_name, i2c_dev, aw_re_f0);
		if (ret < 0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]f0=%dHz\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].f0);
		}
	} else if (strcmp(*(fops_cmd + 3), cmd_type) == 0) {
		ret = awinic_q_cali(cmd, dev_name, i2c_dev, aw_re_f0);
		if (ret < 0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]q=%f\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].q);
		}
	} else if (strcmp(*(fops_cmd + 4), cmd_type) == 0) {
		ret = awinic_fast_start_cali(cmd, dev_name, i2c_dev, aw_re_f0, time);
		if (ret < 0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]fixed_re=%dmohm, f0=%dHz, q=%f\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].fixed_re, aw_re_f0[i].f0, aw_re_f0[i].q);
		}
	} else if (strcmp(*(fops_cmd + 5), cmd_type) == 0) {
		ret = awinic_fast_re_cali(cmd, dev_name, i2c_dev, aw_re_f0, time);
		if (ret <0)
			return ret;
		for (i = 0; i < i2c_dev.dev_num; i++) {
			printf("[%d-00%d]fixed_re=%dmohm\n",
				i2c_dev.i2c_info[i].bus, i2c_dev.i2c_info[i].addr,
				aw_re_f0[i].fixed_re);
		}
	} else if (strcmp(*(fops_cmd + 6), cmd_type) == 0) {
		if (i2c_dev.dev_num != 1) {
			printf("dev_num:%d not comply with the rules\n",
				i2c_dev.dev_num);
			return -1;
		}

		ret = awinic_store_fixed_re(cmd, store_fixed_re);
		if (ret < 0)
			return ret;
	}

	return 0;
}
