/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
uint8_t FCU_state[12];
uint8_t FCU_state_temp[12];
uint8_t diff[12];
char buffer[100];

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define POT_INT_Pin GPIO_PIN_0
#define POT_INT_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define NSS_Reset_Pin GPIO_PIN_7
#define NSS_Reset_GPIO_Port GPIOA
#define NSS_Switch_Pin GPIO_PIN_10
#define NSS_Switch_GPIO_Port GPIOB
#define NSS_AffD_Pin GPIO_PIN_7
#define NSS_AffD_GPIO_Port GPIOC
#define NSS_AffG_Pin GPIO_PIN_8
#define NSS_AffG_GPIO_Port GPIOA
#define NSS_AffC_Pin GPIO_PIN_9
#define NSS_AffC_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define NSS_BL_Pin GPIO_PIN_6
#define NSS_BL_GPIO_Port GPIOB

// API SWITCHS

#define API_SW_FCU_CPT_FD 149
#define API_SW_FCU_CPT_LS 150
#define API_SW_FCU_CPT_MODE_ILS 151
#define API_SW_FCU_CPT_MODE_VOR 152
#define API_SW_FCU_CPT_MODE_NAV 153
#define API_SW_FCU_CPT_MODE_ARC 154
#define API_SW_FCU_CPT_MODE_PLAN 155
#define API_SW_FCU_CPT_RANGE_10 156
#define API_SW_FCU_CPT_RANGE_20 157
#define API_SW_FCU_CPT_RANGE_40 158
#define API_SW_FCU_CPT_RANGE_80 159
#define API_SW_FCU_CPT_RANGE_160 160
#define API_SW_FCU_CPT_RANGE_320 161
#define API_SW_FCU_CPT_ADF1 162
#define API_SW_FCU_CPT_VOR1 163
#define API_SW_FCU_CPT_ADF2 164
#define API_SW_FCU_CPT_VOR2 165
#define API_SW_FCU_CPT_inHg 166
#define API_SW_FCU_CPT_mb 167
#define API_SW_FCU_CPT_CSTR 168
#define API_SW_FCU_CPT_WPT 169
#define API_SW_FCU_CPT_VORDME 170
#define API_SW_FCU_CPT_NDB 171
#define API_SW_FCU_CPT_ARPT 172
#define API_SW_FCU_CPT_QNHinc 173
#define API_SW_FCU_CPT_QNHdec 174
#define API_SW_FCU_CPT_QNHpull 175
#define API_SW_FCU_CPT_QNHpush 176
#define API_SW_FCU_FO_FD 177
#define API_SW_FCU_FO_LS 178
#define API_SW_FCU_FO_MODE_ILS 179
#define API_SW_FCU_FO_MODE_VOR 180
#define API_SW_FCU_FO_MODE_NAV 181
#define API_SW_FCU_FO_MODE_ARC 182
#define API_SW_FCU_FO_MODE_PLAN 183
#define API_SW_FCU_FO_RANGE_10 184
#define API_SW_FCU_FO_RANGE_20 185
#define API_SW_FCU_FO_RANGE_40 186
#define API_SW_FCU_FO_RANGE_80 187
#define API_SW_FCU_FO_RANGE_160 188
#define API_SW_FCU_FO_RANGE_320 189
#define API_SW_FCU_FO_ADF1 190
#define API_SW_FCU_FO_VOR1 191
#define API_SW_FCU_FO_ADF2 192
#define API_SW_FCU_FO_VOR2 193
#define API_SW_FCU_FO_inHg 194
#define API_SW_FCU_FO_mb 195
#define API_SW_FCU_FO_CSTR 196
#define API_SW_FCU_FO_WPT 197
#define API_SW_FCU_FO_VORDME 198
#define API_SW_FCU_FO_NDB 199
#define API_SW_FCU_FO_ARPT 200
#define API_SW_FCU_FO_QNHinc 201
#define API_SW_FCU_FO_QNHdec 202
#define API_SW_FCU_FO_QNHpull 203
#define API_SW_FCU_FO_QNHpush 204
#define API_SW_FCU_TRK_FPA 205
#define API_SW_FCU_METRICALT 206
#define API_SW_FCU_AP1 207
#define API_SW_FCU_AP2 208
#define API_SW_FCU_ATHR 209
#define API_SW_FCU_LOC 210
#define API_SW_FCU_APPR 211
#define API_SW_FCU_EXPED 212
#define API_SW_FCU_SPDpush 213
#define API_SW_FCU_SPDpull 214
#define API_SW_FCU_HDGpush 215
#define API_SW_FCU_HDGpull 216
#define API_SW_FCU_ALTpush 217
#define API_SW_FCU_ALTpull 218
#define API_SW_FCU_VSpush 219
#define API_SW_FCU_VSpull 220
#define API_SW_FCU_ALT100 221
#define API_SW_FCU_SPDinc 222
#define API_SW_FCU_HDGinc 223
#define API_SW_FCU_ALTinc 224
#define API_SW_FCU_VSinc 225
#define API_SW_FCU_SPDdec 226
#define API_SW_FCU_HDGdec 227
#define API_SW_FCU_ALTdec 228
#define API_SW_FCU_VSdec 229
#define API_SW_FCU_ALT1000 378
#define API_SW_FCU_SPD_MACH 379// Inutilisé
#define API_SW_FCU_SPDincBIG 483// Inutilisé
#define API_SW_FCU_HDGincBIG 484// Inutilisé
#define API_SW_FCU_ALTincBIG 485// Inutilisé
#define API_SW_FCU_VSincBIG 486// Inutilisé
#define API_SW_FCU_SPDdecBIG 487// Inutilisé
#define API_SW_FCU_HDGdecBIG 488// Inutilisé
#define API_SW_FCU_ALTdecBIG 489// Inutilisé
#define API_SW_FCU_VSdecBIG 490// Inutilisé
#define API_SW_FCU_CPT_QNHincBIG 491// Inutilisé
#define API_SW_FCU_CPT_QNHdecBIG 493// Inutilisé
#define API_SW_FCU_FO_QNHincBIG 494// Inutilisé
#define API_SW_FCU_FO_QNHdecBIG 495// Inutilisé

// API LEDS

#define API_LED_FCU_CPT_FD 300// Inutilisé
#define API_LED_FCU_CPT_LS 301// Inutilisé
#define API_LED_FCU_CPT_CSTR 302// Inutilisé
#define API_LED_FCU_CPT_WPT 303// Inutilisé
#define API_LED_FCU_CPT_VOR 304// Inutilisé
#define API_LED_FCU_CPT_NDB 305// Inutilisé
#define API_LED_FCU_CPT_ARPT 306// Inutilisé
#define API_LED_FCU_CPT_QFE 307// Inutilisé
#define API_LED_FCU_CPT_QNH 308// Inutilisé
#define API_LED_FCU_SPD 309// Inutilisé
#define API_LED_FCU_MACH 310// Inutilisé
#define API_LED_FCU_HDGVS 311// Inutilisé
#define API_LED_FCU_TRKFPA 312// Inutilisé
#define API_LED_FCU_HDGDOT 314// Inutilisé
#define API_LED_FCU_ALTDOT 322// Inutilisé
#define API_LED_FCU_SPDDOT 323// Inutilisé
#define API_LED_FCU_AP1 324// Inutilisé
#define API_LED_FCU_AP2 325// Inutilisé
#define API_LED_FCU_ATHR 326// Inutilisé
#define API_LED_FCU_LOC 327// Inutilisé
#define API_LED_FCU_APPR 328// Inutilisé
#define API_LED_FCU_EXPED 329// Inutilisé
#define API_LED_FCU_FO_FD 330// Inutilisé
#define API_LED_FCU_FO_LS 331// Inutilisé
#define API_LED_FCU_FO_CSTR 332// Inutilisé
#define API_LED_FCU_FO_WPT 333// Inutilisé
#define API_LED_FCU_FO_VOR 334// Inutilisé
#define API_LED_FCU_FO_NDB 335// Inutilisé
#define API_LED_FCU_FO_ARPT 336// Inutilisé
#define API_LED_FCU_FO_QFE 337// Inutilisé
#define API_LED_FCU_FO_QNH 338// Inutilisé
#define API_LED_FCU_CPT_QNHdec 339// Inutilisé
#define API_LED_FCU_FO_QNHdec 340// Inutilisé

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

void Serial_Transmit(char* str);
int bitRead(uint8_t byte, int nb);
int bitSet(uint8_t byte, int nb);
int bitClear(uint8_t byte, int nb);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
