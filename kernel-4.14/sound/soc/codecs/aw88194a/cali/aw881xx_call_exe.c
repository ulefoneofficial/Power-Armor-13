#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <sys/wait.h>

static char *fops_cmd[] = {
	"start_cali",
	"re_cali",
	"f0_cali",
	"store_re",
	"store_fixed_re",
	"fast_start_cali",
	"fast_re_cali",
};

void awinic_call_exe(char *buf, char *cmd)
{
	int ret;
	int fd[2];
	pid_t pid;
	char *ret_str;

	if (pipe(fd) < 0) {
		perror("pipe error");
		exit(1);
	}

	pid = fork();
	if (pid == 0) {
		if (dup2(fd[1], STDOUT_FILENO) != STDOUT_FILENO) {
			perror("dup2 error");
		}
		system(cmd);
	} else if (pid > 0) {
		ret = read(fd[0], buf, 500);
		printf("num:%d\ninfo:\n%s", ret, buf);
		wait(NULL);
	} else {
		printf("fork failed\n");
	}

}


void  awinic_connect_cmd(char *cmd, char *add_cmd)
{
	while (*cmd != '\0')
		cmd++;

	while (*add_cmd != '\0')
		*cmd++ = *add_cmd++;

	add_cmd--;
	if (*add_cmd != ' ') {
		*cmd++ = ' ';
	}

}

int awinic_assemble_cmd(char *cmd, char *exe_name, char *cmd_type,
				char *dev_name, int i2c_bus, int i2c_addr)
{
	int mov = 0;
	char buf[6] = {0};

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

	snprintf(buf, 6, "0x%02x", i2c_bus);
	awinic_connect_cmd(cmd, buf);

	snprintf(buf, 6, "0x%02x", i2c_addr);
	awinic_connect_cmd(cmd, buf);

	return 0;
	}



int awinic_found_string(char *buf, char *front_str, char *end_str, char *str)
{
	char *front_p;
	char *end_p;

	front_p = strstr(buf, front_str);
	if (!front_p) {
		printf("%s not found\n", front_str);
		return -1;
	}
	while (*front_str != '\0') {
		front_p++;
		front_str++;
	}

	if (end_str) {
		end_p = strstr(buf, end_str);
		 if (!end_p) {
			printf("%s not found\n", end_str);
		 return -1;
		}
		while (front_p != end_p)
			*str++ = *front_p++;
	} else {
		while (*front_p != '\0')
			*str++ = *front_p++;
	}

	*str = '\0';

	return 0;
	}


void awinic_start_cali(char *cmd, float *re, int *fixed_re, float *f0)
{
	int ret;
	char buf[500] = {0};
	char re_str[20] = {0}, fixed_re_str[20] = {0}, f0_str[20] = {0};

	*re = 0;
	*f0 = 0;
	*fixed_re = 0;
	printf("%s\n", cmd);

	awinic_call_exe(buf, cmd);

	ret = awinic_found_string(buf, "re=", "ohm", re_str);
	if (ret < 0)
		return;
	*re = (float)atof(re_str);

	ret = awinic_found_string(buf, "fixed_re=", "mohm", fixed_re_str);
	if (ret < 0)
		return;
	*fixed_re = atoi(fixed_re_str);

	ret = awinic_found_string(buf, "f0=", NULL, f0_str);
	if (ret < 0)
		return;
	*f0 = (float)atof(f0_str);

}

void awinic_re_cali(char *cmd, float *re, int *fixed_re)
{
	int ret;
	char buf[500] = {0};
	char re_str[20] = {0}, fixed_re_str[20] = {0};

	*re = 0;
	*fixed_re = 0;
	printf("%s\n", cmd);
	awinic_call_exe(buf, cmd);

	ret = awinic_found_string(buf, "re=", "ohm", re_str);
	if (ret < 0)
		return;
	*re = (float)atof(re_str);

	ret = awinic_found_string(buf, "fixed_re=", "mohm", fixed_re_str);
	if (ret < 0)
		return;
	*fixed_re = atoi(fixed_re_str);

}

void awinic_f0_cali(char *cmd, float *f0)
{
	int ret;
	char buf[500] = {0};
	char f0_str[20] = {0};

	*f0 = 0;
	printf("%s\n", cmd);
	awinic_call_exe(buf, cmd);

	ret = awinic_found_string(buf, "f0=", NULL, f0_str);
	if (ret < 0)
		return;
	*f0 = (float)atof(f0_str);

}

void awinic_store_re(char *cmd, float re)
{
	char re_buf[16] = {0};
	char buf[500] = {0};

	if (re < 0.000001) {
		printf("re not set, re value:%f\n", re);
		return;
	}

	snprintf(re_buf, 16, "%f", re);
	awinic_connect_cmd(cmd, re_buf);
	printf("%s\n", cmd);

	awinic_call_exe(buf, cmd);

}

void awinic_store_fixed_re(char *cmd, int fixed_re)
{
	char fixed_re_buf[16] = {0};
	char buf[500] = {0};

	if (!fixed_re) {
		printf("re not set, re value:%d\n", fixed_re);
		return;
	}

	snprintf(fixed_re_buf, 16, "%d", fixed_re);
	awinic_connect_cmd(cmd, fixed_re_buf);
	printf("%s\n", cmd);

	awinic_call_exe(buf, cmd);

}

void awinic_fast_start_cali(char *cmd,
			float *re, int *fixed_re, float *f0, int time)
{
	int ret;
	char buf[500] = {0};
	char re_str[20] = {0}, fixed_re_str[20] = {0}, f0_str[20] = {0};
	char time_buf[16] = {0};

	*re = 0;
	*f0 = 0;
	*fixed_re = 0;

	snprintf(time_buf, 16, "%d", time);
	awinic_connect_cmd(cmd, time_buf);
	printf("%s\n", cmd);

	awinic_call_exe(buf, cmd);

	ret = awinic_found_string(buf, "re=", "ohm", re_str);
	if (ret < 0)
		return;
	*re = (float)atof(re_str);

	ret = awinic_found_string(buf, "fixed_re=", "mohm", fixed_re_str);
	if (ret < 0)
		return;
	*fixed_re = atoi(fixed_re_str);

	ret = awinic_found_string(buf, "f0=", NULL, f0_str);
	if (ret < 0)
		return;
	*f0 = (float)atof(f0_str);

}

void awinic_fast_re_cali(char *cmd,
		float *re, int *fixed_re, int time)
{
	int ret;
	char buf[500] = {0};
	char re_str[20] = {0}, fixed_re_str[20] = {0};
	char time_buf[16] = {0};

	*re = 0;
	*fixed_re = 0;

	snprintf(time_buf, 16, "%d", time);
	awinic_connect_cmd(cmd, time_buf);
	printf("%s\n", cmd);
	awinic_call_exe(buf, cmd);

	ret = awinic_found_string(buf, "re=", "ohm", re_str);
	if (ret < 0)
		return;
	*re = (float)atof(re_str);

	ret = awinic_found_string(buf, "fixed_re=", "mohm", fixed_re_str);
	if (ret < 0)
		return;
	*fixed_re = atoi(fixed_re_str);

}

int main(int argc, char *argv[])
{
	int ret;
	char cmd[100] = {0};
	float f0 = 0;

	/*must write*/
	char *exe_name = "aw881xx_cali";
	char *cmd_type = fops_cmd[6];
	int i2c_bus = 0x06;
	int i2c_addr = 0x34;
	char dev_name[30] = {"aw881xx_smartpa"};

	/*store_re must set*/
	float re = 8.00;

	/*store_fixed_re must set*/
	int fixed_re = 8000;

	/*fast_start_cali and fast_re_cali must set*/
	int time = 10000;

	ret = awinic_assemble_cmd(cmd, exe_name, cmd_type,
						dev_name, i2c_bus, i2c_addr);
	if (ret < 0)
		return -1;

	if (strcmp(*(fops_cmd + 0), cmd_type) == 0) {
		awinic_start_cali(cmd, &re, &fixed_re, &f0);
		printf("re=%f, fixed_re=%d, f0=%f\n", re, fixed_re, f0);
	} else if (strcmp(*(fops_cmd + 1), cmd_type) == 0) {
		awinic_re_cali(cmd, &re, &fixed_re);
		printf("re=%f, fixed_re=%d\n",re, fixed_re);
	} else if (strcmp(*(fops_cmd + 2), cmd_type) == 0) {
		awinic_f0_cali(cmd, &f0);
		printf("f0=%f\n", f0);
	} else if (strcmp(*(fops_cmd + 3), cmd_type) == 0) {
		awinic_store_re(cmd, re);
	} else if (strcmp(*(fops_cmd + 4), cmd_type) == 0) {
		awinic_store_fixed_re(cmd, fixed_re);
	} else if (strcmp(*(fops_cmd + 5), cmd_type) == 0) {
		awinic_fast_start_cali(cmd, &re, &fixed_re, &f0, time);
		printf("re=%f, fixed_re=%d, f0=%f\n", re, fixed_re, f0);
	} else if (strcmp(*(fops_cmd + 6), cmd_type) == 0) {
		awinic_fast_re_cali(cmd, &re, &fixed_re, time);
		printf("re=%f, fixed_re=%d\n",re, fixed_re);
	}

	return 0;
}
