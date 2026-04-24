# 学习UART

## 低级UART：阻塞cpu控制（青铜）

UART（Universal Asynchronous Receiver/Transmitter）是一种常见的串行通信协议，用于在计算机和外部设备之间进行数据传输。在低级UART中，CPU通过阻塞方式控制数据的发送和接收。

在阻塞模式下，CPU会等待数据的发送或接收完成后才继续执行其他任务。这意味着在数据传输过程中，CPU无法进行其他操作，导致系统性能下降。

### 简单的示例代码，展示了如何使用阻塞方式控制UART通信：

```c
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#define UART_DEVICE "/dev/ttyS0" // UART设备路径

int main() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        perror("Failed to open UART device");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B9600); // 设置输入波特率
    cfsetospeed(&options, B9600); // 设置输出波特率
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收器
    tcsetattr(uart_fd, TCSANOW, &options);

    // 发送数据
    const char *data_to_send = "Hello UART!";
    write(uart_fd, data_to_send, strlen(data_to_send));

    // 接收数据
    char buffer[100];
    ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0'; // 添加字符串结束符
        printf("Received: %s\n", buffer);
    }

    close(uart_fd);
    return 0;
}
```
在这个示例中，CPU通过`write`函数发送数据，并通过`read`函数接收数据。在阻塞模式下，CPU会等待这些操作完成后才继续执行其他任务。

虽然这种方式简单易懂，但在实际应用中可能会导致性能问题，特别是在需要同时处理多个任务的情况下。因此，在更高级的UART通信中，通常会使用非阻塞模式或中断驱动方式来提高系统效率。


## 高级UART：非阻塞cpu控制（白银）
在高级UART中，CPU通过非阻塞方式控制数据的发送和接收。这意味着CPU可以在等待数据传输完成的同时执行其他任务，从而提高系统性能。

### 示例代码，展示了如何使用非阻塞方式控制UART通信：

```c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#define UART_DEVICE "/dev/ttyS0" // UART设备路径

int main() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        perror("Failed to open UART device");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B9600); // 设置输入波特率
    cfsetospeed(&options, B9600); // 设置输出波特率
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收器
    tcsetattr(uart_fd, TCSANOW, &options);

    // 发送数据
    const char *data_to_send = "Hello UART!";
    write(uart_fd, data_to_send, strlen(data_to_send));

    // 非阻塞接收数据
    char buffer[100];
    ssize_t bytes_read;
    while (1) {
        bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // 添加字符串结束符
            printf("Received: %s\n", buffer);
        }
        // 在这里可以执行其他任务，而不必等待数据传输完成
        usleep(100000); // 模拟其他任务的处理时间
    }

    close(uart_fd);
    return 0;
}
```
在这个示例中，CPU通过`O_NONBLOCK`标志打开UART设备，使得`read`函数在没有数据可读时不会阻塞CPU。CPU可以在等待数据传输完成的同时执行其他任务，从而提高系统效率。
虽然非阻塞方式可以提高性能，但需要注意处理好数据的同步和错误情况，以确保系统的稳定性和可靠性。

## 中断驱动UART：黄金
在中断驱动UART中，CPU通过中断机制控制数据的发送和接收。当UART设备准备好发送或接收数据时，会触发一个中断，CPU响应中断并处理数据传输。这种方式可以最大程度地提高系统性能，因为CPU只在需要处理数据时才会被打扰。

### 示例代码，展示了如何使用中断驱动方式控制UART通信：
```c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#define UART_DEVICE "/dev/ttyS0" // UART设备路径
volatile sig_atomic_t data_received = 0;
void uart_interrupt_handler(int signum) {
    data_received = 1; // 设置标志，表示数据已接收
}
int main() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        perror("Failed to open UART device");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B9600); // 设置输入波特率
    cfsetospeed(&options, B9600); // 设置输出波特率
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收器
    tcsetattr(uart_fd, TCSANOW, &options);

    // 注册中断处理程序
    signal(SIGIO, uart_interrupt_handler);
    fcntl(uart_fd, F_SETOWN, getpid());
    fcntl(uart_fd, F_SETFL, FASYNC);

    // 发送数据
    const char *data_to_send = "Hello UART!";
    write(uart_fd, data_to_send, strlen(data_to_send));

    while (1) {
        if (data_received) {
            char buffer[100];
            ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0'; // 添加字符串结束符
                printf("Received: %s\n", buffer);
            }
            data_received = 0; // 重置标志
        }
        // 在这里可以执行其他任务，而不必等待数据传输完成
        usleep(100000); // 模拟其他任务的处理时间
    }

    close(uart_fd);
    return 0;
}
```
在这个示例中，CPU通过注册一个信号处理程序来响应UART设备的中断。当UART设备准备好发送或接收数据时，会触发一个`SIGIO`信号，CPU响应该信号并执行`uart_interrupt_handler`函数来处理数据传输。这种方式可以最大程度地提高系统性能，因为CPU只在需要处理数据时才会被打扰。

虽然中断驱动方式可以提供最佳性能，但需要注意正确处理中断和同步问题，以确保系统的稳定性和可维护性。


