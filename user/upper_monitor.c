#include "upper_monitor.h"
/*
 * 山外多功能调试助手上位机，虚拟示波器显示函数
 * wareaddr    波形数组起始地址
 * waresize    波形数组占用空间的大小
 * example:
 *        uint32_t var[4]={1,2,3,4};
 *        vcan_sendware((uint8_t *)var,sizeof(var));
 *
 */
void vcan_sendware( uint8_t *wareaddr, uint32_t waresize )
{
#define CMD_WARE 3
	uint8_t cmdf[2] = { CMD_WARE, ~CMD_WARE };      /* 串口调试 使用的前命令 */
	uint8_t cmdr[2] = { ~CMD_WARE, CMD_WARE };      /* 串口调试 使用的后命令 */
	uart_putbuff( HW_UART3, cmdf, sizeof(cmdf) );   /* 先发送前命令 */
	uart_putbuff( HW_UART3, wareaddr, waresize );   /* 发送数据 */
	uart_putbuff( HW_UART3, cmdr, sizeof(cmdr) );   /* 发送后命令 */
}


/*
 * 山外多功能调试助手上位机，摄像头显示函数
 * imgaddr    图像起始地址
 * imgsize    图像占用空间的大小
 * example:   vcan_sendimg(imgadd,row_num*col_num);   imgadd为需要传输的图像数据
 */
void vcan_sendimg( uint8_t *imgaddr, uint32_t imgsize )
{
#define CMD_IMG 1
	uint8_t cmdf[2] = { CMD_IMG, ~CMD_IMG };                /* 山外上位机 使用的命令 */
	uint8_t cmdr[2] = { ~CMD_IMG, CMD_IMG };                /* 山外上位机 使用的命令 */

	uart_putbuff( HW_UART3, cmdf, sizeof(cmdf) );           /* 先发送命令 */

	uart_putbuff( HW_UART3, (char *) imgaddr, imgsize );    /* 再发送图像 */

	uart_putbuff( HW_UART3, cmdr, sizeof(cmdr) );           /* 先发送命令 */
}