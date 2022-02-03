/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* GPIO_AD_25 (coord M15), LPUART1_RXD */
/* Routed pin properties */
#define BOARD_INITPINS_LPUART1_RXD_PERIPHERAL                            LPUART1   /*!< Peripheral name */
#define BOARD_INITPINS_LPUART1_RXD_SIGNAL                                    RXD   /*!< Signal name */

/* GPIO_AD_24 (coord L13), LPUART1_TXD */
/* Routed pin properties */
#define BOARD_INITPINS_LPUART1_TXD_PERIPHERAL                            LPUART1   /*!< Peripheral name */
#define BOARD_INITPINS_LPUART1_TXD_SIGNAL                                    TXD   /*!< Signal name */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);                    /* Function assigned for the Cortex-M7F */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPI2C1_InitPins(void);                   /* Function assigned for the Cortex-M7F */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPI2C1_DeinitPins(void);                 /* Function assigned for the Cortex-M7F */

/* GPIO_AD_29 (coord M17), LPSPI1_PCS0/U27[1]/J10[6] */
/* Routed pin properties */
#define LPSPI1_INITPINS_LPSPI1_PCS0_PERIPHERAL                            LPSPI1   /*!< Peripheral name */
#define LPSPI1_INITPINS_LPSPI1_PCS0_SIGNAL                                  PCS0   /*!< Signal name */

/* GPIO_AD_28 (coord L17), LPSPI1_SCK/U27[6]/BT_DEV_WAKE/U354[9]/J10[12] */
/* Routed pin properties */
#define LPSPI1_INITPINS_LPSPI1_SCK_PERIPHERAL                             LPSPI1   /*!< Peripheral name */
#define LPSPI1_INITPINS_LPSPI1_SCK_SIGNAL                                    SCK   /*!< Signal name */

/* GPIO_AD_31 (coord J17), LPSPI1_SDI/U27[2]/J10[10] */
/* Routed pin properties */
#define LPSPI1_INITPINS_LPSPI1_SDI_PERIPHERAL                             LPSPI1   /*!< Peripheral name */
#define LPSPI1_INITPINS_LPSPI1_SDI_SIGNAL                                    SIN   /*!< Signal name */

/* GPIO_AD_30 (coord K17), LPSPI1_SDO/U27[5]/Backlight_CTL/J48[34]/J10[8] */
/* Routed pin properties */
#define LPSPI1_INITPINS_LPSPI1_SDO_PERIPHERAL                             LPSPI1   /*!< Peripheral name */
#define LPSPI1_INITPINS_LPSPI1_SDO_SIGNAL                                   SOUT   /*!< Signal name */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPSPI1_InitPins(void);                   /* Function assigned for the Cortex-M7F */

/* GPIO_AD_29 (coord M17), LPSPI1_PCS0/U27[1]/J10[6] */
/* Routed pin properties */
#define LPSPI1_DEINITPINS_LPSPI1_PCS0_PERIPHERAL                           GPIO3   /*!< Peripheral name */
#define LPSPI1_DEINITPINS_LPSPI1_PCS0_SIGNAL                         gpio_mux_io   /*!< Signal name */
#define LPSPI1_DEINITPINS_LPSPI1_PCS0_CHANNEL                                28U   /*!< Signal channel */

/* GPIO_AD_28 (coord L17), LPSPI1_SCK/U27[6]/BT_DEV_WAKE/U354[9]/J10[12] */
/* Routed pin properties */
#define LPSPI1_DEINITPINS_LPSPI1_SCK_PERIPHERAL                            GPIO3   /*!< Peripheral name */
#define LPSPI1_DEINITPINS_LPSPI1_SCK_SIGNAL                          gpio_mux_io   /*!< Signal name */
#define LPSPI1_DEINITPINS_LPSPI1_SCK_CHANNEL                                 27U   /*!< Signal channel */

/* GPIO_AD_31 (coord J17), LPSPI1_SDI/U27[2]/J10[10] */
/* Routed pin properties */
#define LPSPI1_DEINITPINS_LPSPI1_SDI_PERIPHERAL                            GPIO3   /*!< Peripheral name */
#define LPSPI1_DEINITPINS_LPSPI1_SDI_SIGNAL                          gpio_mux_io   /*!< Signal name */
#define LPSPI1_DEINITPINS_LPSPI1_SDI_CHANNEL                                 30U   /*!< Signal channel */

/* GPIO_AD_30 (coord K17), LPSPI1_SDO/U27[5]/Backlight_CTL/J48[34]/J10[8] */
/* Routed pin properties */
#define LPSPI1_DEINITPINS_LPSPI1_SDO_PERIPHERAL                            GPIO3   /*!< Peripheral name */
#define LPSPI1_DEINITPINS_LPSPI1_SDO_SIGNAL                          gpio_mux_io   /*!< Signal name */
#define LPSPI1_DEINITPINS_LPSPI1_SDO_CHANNEL                                 29U   /*!< Signal channel */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPSPI1_DeinitPins(void);                 /* Function assigned for the Cortex-M7F */

/* GPIO_AD_25 (coord M15), LPUART1_RXD */
/* Routed pin properties */
#define LPUART1_INITPINS_LPUART1_RXD_PERIPHERAL                          LPUART1   /*!< Peripheral name */
#define LPUART1_INITPINS_LPUART1_RXD_SIGNAL                                  RXD   /*!< Signal name */

/* GPIO_AD_24 (coord L13), LPUART1_TXD */
/* Routed pin properties */
#define LPUART1_INITPINS_LPUART1_TXD_PERIPHERAL                          LPUART1   /*!< Peripheral name */
#define LPUART1_INITPINS_LPUART1_TXD_SIGNAL                                  TXD   /*!< Signal name */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPUART1_InitPins(void);                  /* Function assigned for the Cortex-M7F */

/* GPIO_AD_25 (coord M15), LPUART1_RXD */
/* Routed pin properties */
#define LPUART1_DEINITPINS_LPUART1_RXD_PERIPHERAL                          GPIO3   /*!< Peripheral name */
#define LPUART1_DEINITPINS_LPUART1_RXD_SIGNAL                        gpio_mux_io   /*!< Signal name */
#define LPUART1_DEINITPINS_LPUART1_RXD_CHANNEL                               24U   /*!< Signal channel */

/* GPIO_AD_24 (coord L13), LPUART1_TXD */
/* Routed pin properties */
#define LPUART1_DEINITPINS_LPUART1_TXD_PERIPHERAL                          GPIO3   /*!< Peripheral name */
#define LPUART1_DEINITPINS_LPUART1_TXD_SIGNAL                        gpio_mux_io   /*!< Signal name */
#define LPUART1_DEINITPINS_LPUART1_TXD_CHANNEL                               23U   /*!< Signal channel */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void LPUART1_DeinitPins(void);                /* Function assigned for the Cortex-M7F */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
