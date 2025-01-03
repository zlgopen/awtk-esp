#ifndef AWTK_CONFIG_H
#define AWTK_CONFIG_H

/**
 * 嵌入式系统有自己的main函数时，请定义本宏。
 *
 */
#define USE_GUI_MAIN 1

/**
 * 如果支持png/jpeg图片，请定义本宏
 *
 */
#define WITH_STB_IMAGE 1

/**
 * 如果支持Truetype字体，请定义本宏
 *
 */
#define WITH_STB_FONT 1

/**
 * 如果定义本宏，使用标准的UNICODE换行算法，除非资源极为有限，请定义本宏。
 *
 */
#define WITH_UNICODE_BREAK 1

/**
 * 如果定义本宏，将图片解码成BGRA8888格式，否则解码成RGBA8888的格式。
 *
 */
#define WITH_BITMAP_BGRA 1

/**
 * 如果定义本宏，将不透明的PNG图片解码成BGR565格式，建议定义。
 *
 */
#define WITH_BITMAP_BGR565 1

/**
 * 如果FLASH空间较小，不足以放大字体文件时，请定义本宏
 *
 */
#define WITH_MINI_FONT 1

 /**
 * 如果启用VGCANVAS，而且没有OpenGL硬件加速，请定义本宏
 *
 */
#define WITH_NANOVG_AGGE 1

/**
 * 如果启用VGCANVAS，请定义本宏
 *
 */
#define WITH_VGCANVAS 1

/**
 * 启用输入法，但不想启用联想功能，请定义本宏。
 *
 */
#define WITHOUT_SUGGEST_WORDS 1

/**
 * 启用输入法，请定义本宏。
 *
 */
#define WITH_IME_NULL 1
#define WITH_NULL_IM 1

#define HAS_STD_MALLOC 1

/**
 * 支持printf函数。
 *
 */
#define HAS_STDIO 1

/**
 * 包含FreeRTOS的头文件时带freertos目录。
 *
 */
#define WITH_FREE_RTOS_DIR 1

#endif/*AWTK_CONFIG_H*/

