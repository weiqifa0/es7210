#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> /*_llseek*/
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#define LENGTH  100
#define TEST "123456789ABCDEF1011121314151617181920"

/*****************************************************
lseek()将文件读写指针相对whence移动offset个字节。
操作成功时，返回文件指针相对于文件头的位置。
参数whence可使用下述值：
SEEK_SET：相对文件开头
SEEK_CUR：相对文件读写指针的当前位置
SEEK_END：相对文件末尾
offset可取负值

https://blog.csdn.net/qq_35733751/article/details/80709783
******************************************************/

int open_and_putout(int fd)
{
    int len;
    char str[LENGTH];
    if(fd < 0)
    {
        printf("file open error\n");
        return -1;
    }

    len = read(fd, str, LENGTH);
    str[len] = '\0';
    printf("str:%s\n", str);
    return 0;
}

int write_data_to_file(int fd)
{
    if(fd < 0)
    {
        printf("file open error\n");
        return -1;
    }
    write(fd, TEST, strlen(TEST));
    printf("write data ok! fd:%d\n",fd);
    return 0;    
}

int memset_fd_zero(int fd)
{
    int ret;
    if(fd < 0)
    {
        printf("file open error\n");
        return -1;
    }
    printf("ioctl globalmem zero\n");
    ioctl(fd,0x01,&ret);
    printf("ret:%d\n",ret);
    return 0;    
}

int main(int argc, char *argv[])
{
    int fd,ret;
    long int length;
    /*打开设备文件*/
    fd = open("/dev/globalmem", O_RDWR);if(fd<0){printf("open er\n");return -1;}

    ret = lseek(fd, 0, SEEK_SET);
    printf("file ret:%d\n",ret);
    write_data_to_file(fd);
    ret = lseek(fd, 0, SEEK_SET);
    open_and_putout(fd);

    ret = lseek(fd, 1, SEEK_SET);
    printf("lseek:%d\n",ret);
    open_and_putout(fd);

    if(fd>0)
        close(fd);
    return 0;
}
