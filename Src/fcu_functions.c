/*
 * fcu_functions.c
 *
 *  Created on: 7 mai 2018
 *      Author: Yael
 */

#include "fcu_functions.h"

uint8_t no_op = 0x00;

/**
 * Transmettre une donn�e � la partie gauche du FCU
 * Param�tres : reg = registre o� envoyer la donn�e
 * 			    data = valeur � envoyer au registre
 */
void FCU_Transmit_G(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_AffG_GPIO_Port, NSS_AffG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_AffG_GPIO_Port, NSS_AffG_Pin, GPIO_PIN_SET);
}

/**
 * Transmettre une donn�e � la partie droite du FCU
 * Param�tres : reg = registre o� envoyer la donn�e
 * 			    data = valeur � envoyer au registre
 */
void FCU_Transmit_D(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_AffD_GPIO_Port, NSS_AffD_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_AffD_GPIO_Port, NSS_AffD_Pin, GPIO_PIN_SET);
}

/**
 * Transmettre une donn�e � la partie centrale du FCU
 * Param�tres : addr = addresse de la sous partie � controler de 1 � 3 ( 0 = toutes )
 * 				reg = registre o� envoyer la donn�e
 * 			    data = valeur � envoyer au registre
 */
void FCU_Transmit_C(uint8_t addr, uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_AffC_GPIO_Port, NSS_AffC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (addr == 3 || addr == 0 ? &reg : &no_op) , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_SPI_Transmit(&hspi3, (addr == 2 || addr == 0 ? &reg : &no_op) , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_SPI_Transmit(&hspi3, (addr == 1 || addr == 0 ? &reg : &no_op) , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_AffC_GPIO_Port, NSS_AffC_Pin, GPIO_PIN_SET);
}

/**
 * Effacer et �teindre tous les digits, leds, et voyants du FCU
 */
void FCU_Affich_Clear()
{
	for(uint8_t i = 0; i < 7; i++)
	{
		FCU_Transmit_G(i, DIGIT_BLANK);
		FCU_Transmit_D(i, DIGIT_BLANK);
		FCU_Transmit_C(0, i, DIGIT_BLANK);
	}
	FCU_Transmit_G(5, 0x00);
	FCU_Transmit_G(6, 0x00);
	FCU_Transmit_C(1, 7, 0x00);
	FCU_Transmit_C(1, 8, 0x00);
	FCU_Transmit_C(2, 6, 0x00);
	FCU_Transmit_C(2, 7, 0x00);
	FCU_Transmit_C(3, 6, 0x00);
	FCU_Transmit_C(3, 7, 0x00);
	FCU_Transmit_D(5, 0x00);
	FCU_Transmit_D(6, 0x00);
}

/**
 * Teste l'adressage et les registres de chaque partie du FCU en les allumant � tour de r�le
 */

void FCU_Affich_Init()
{
	  FCU_Affich_Clear();
	  FCU_Transmit_G(SHUTDOWN_MODE, 0x01); // No shutdown
	  FCU_Transmit_G(SCAN_LIMIT, 0x05); // Scan Limit 0-5
	  FCU_Transmit_G(DECODE_MODE, 0x0F); // Decode Mode, digits 3-0
	  FCU_Transmit_G(INTENSITY, 0x0E); // Intensit� �lev�e

	  FCU_Transmit_G(TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_G(TEST_DISPLAY, 0x00);

	  FCU_Transmit_C(0, SHUTDOWN_MODE, 0x01); // No shutdown
	  FCU_Transmit_C(0, SCAN_LIMIT, 0x06); // Scan Limit 0-6
	  FCU_Transmit_C(1, SCAN_LIMIT, 0x07); // Scan Limit 0-7
	  FCU_Transmit_C(0, DECODE_MODE, 0x1F); // Decode Mode, digits 4-0
	  FCU_Transmit_C(1, DECODE_MODE, 0x3F); // Decode Mode, digits 5-0
	  FCU_Transmit_C(0, INTENSITY, 0x0E); // Intensit� �lev�e
	  FCU_Transmit_C(1, TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_C(1, TEST_DISPLAY, 0x00);
	  FCU_Transmit_C(2, TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_C(2, TEST_DISPLAY, 0x00);
	  FCU_Transmit_C(3, TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_C(3, TEST_DISPLAY, 0x00);
	  FCU_Transmit_C(0, TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_C(0, TEST_DISPLAY, 0x00);

	  FCU_Transmit_D(SHUTDOWN_MODE, 0x01); // No shutdown
	  FCU_Transmit_D(SCAN_LIMIT, 0x05); // Scan Limit 0-5
	  FCU_Transmit_D(DECODE_MODE, 0x0F); // Decode Mode, digits 3-0
	  FCU_Transmit_D(INTENSITY, 0x0E); // Intensit� �lev�e
	  FCU_Transmit_D(TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_D(TEST_DISPLAY, 0x00);
}
