/*
 * fcu_functions.c
 *
 *  Created on: 7 mai 2018
 *      Author: Yael
 */

#include "fcu_functions.h"

uint8_t no_op = 0x00;


/**
 * Transmettre une donnée aux afficheurs de la partie gauche du FCU
 * Paramètres : reg = registre où envoyer la donnée (Digits 1-4, leds 5, indics 6)
 * 			    data = valeur à envoyer au registre ( defines dans fcu_functions.h )
 */
void FCU_Transmit_G(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_AffG_GPIO_Port, NSS_AffG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_AffG_GPIO_Port, NSS_AffG_Pin, GPIO_PIN_SET);
}

/**
 * Transmettre une donnée aux afficheurs de la partie droite du FCU
 * Paramètres : reg = registre où envoyer la donnée (Digits 1-4, leds 5, indics 8)
 * 			    data = valeur à envoyer au registre ( defines dans fcu_functions.h )
 */
void FCU_Transmit_D(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_AffD_GPIO_Port, NSS_AffD_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_AffD_GPIO_Port, NSS_AffD_Pin, GPIO_PIN_SET);
}

/**
 * Transmettre une donnée aux afficheurs de la partie centrale du FCU
 * Paramètres : addr = addresse de la sous partie à controler de 1 à 3 ( 0 = toutes )
 * 				reg = registre où envoyer la donnée (Digits 1-5 (6 pour addr = 1), leds & indics 6-7 (7-8 pour addr = 1))
 * 			    data = valeur à envoyer au registre ( defines dans fcu_functions.h )
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
 * Effacer et éteindre tous les digits, leds, et voyants du FCU
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
 * Teste l'adressage et les registres de chaque partie du FCU en les allumant à tour de rôle
 */
void FCU_Affich_Init()
{
	  FCU_Affich_Clear();
	  FCU_Transmit_G(SHUTDOWN_MODE, 0x01); // No shutdown
	  FCU_Transmit_G(SCAN_LIMIT, 0x05); // Scan Limit 0-5
	  FCU_Transmit_G(DECODE_MODE, 0x0F); // Decode Mode, digits 3-0
	  FCU_Transmit_G(INTENSITY, 0x0E); // Intensité élevée

	  FCU_Transmit_G(TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_G(TEST_DISPLAY, 0x00);

	  FCU_Transmit_C(0, SHUTDOWN_MODE, 0x01); // No shutdown
	  FCU_Transmit_C(0, SCAN_LIMIT, 0x06); // Scan Limit 0-6
	  FCU_Transmit_C(1, SCAN_LIMIT, 0x07); // Scan Limit 0-7
	  FCU_Transmit_C(0, DECODE_MODE, 0x1F); // Decode Mode, digits 4-0
	  FCU_Transmit_C(1, DECODE_MODE, 0x3F); // Decode Mode, digits 5-0
	  FCU_Transmit_C(0, INTENSITY, 0x0E); // Intensité élevée
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
	  FCU_Transmit_D(INTENSITY, 0x0E); // Intensité élevée
	  FCU_Transmit_D(TEST_DISPLAY, 0x01);
	  HAL_Delay(500);
	  FCU_Transmit_D(TEST_DISPLAY, 0x00);
}

/**
 * Transmettre une donnée aux registres des switchs
 * Paramètres : addr = addresse physique de la puce (0-5)
 * 				reg = registre où envoyer la donnée (defines dans fcu_functions.h)
 * 			    data = valeur à envoyer au registre
 */
void FCU_TransmitSW(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t final_addr = OPCODEW | (addr << 1u);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &final_addr , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_SET);
}

/**
 * Recevoir une donnée des registres des switchs
 * Paramètres : addr = addresse physique de la puce (0-5)
 * 				reg = registre d'où recevoir la donnée (defines dans fcu_functions.h)
 */
uint8_t FCU_ReceiveSW(uint8_t addr, uint8_t reg)
{
	uint8_t value = 0;
	uint8_t final_addr = OPCODER | (addr << 1u);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &final_addr , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_TransmitReceive(&hspi3, &value , &value, 1, 1000);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_SET);
	return value;
}

/**
 * Initialise les registres des switchs
 */
void FCU_Switchs_Init()
{
	for(int ad = 0; ad < 6; ad++)
	{
		FCU_TransmitSW(ad, IOCON, 0x08);
		FCU_TransmitSW(ad, IODIRA, 0xFF);
		FCU_TransmitSW(ad, IODIRB, 0xFF);
		FCU_TransmitSW(ad, GPPUA, 0xFF);
		FCU_TransmitSW(ad, GPPUB, 0xFF);
	}
}

/**
 * Initialise le tableau qui va contenir toutes les valeures attendues par le simulateur
 */
void FCU_API_Init()
{
	API_data[0][0] = API_SW_FCU_CPT_RANGE_320;
	API_data[0][1] = API_SW_FCU_CPT_RANGE_160;
	API_data[0][2] = API_SW_FCU_CPT_RANGE_10;
	API_data[0][4] = API_SW_FCU_CPT_MODE_PLAN;
	API_data[0][5] = API_SW_FCU_CPT_MODE_VOR;
	API_data[0][6] = API_SW_FCU_CPT_MODE_ARC;
	API_data[0][7] = API_SW_FCU_CPT_MODE_NAV;

	API_data[1][0] = API_SW_FCU_CPT_CSTR;
	API_data[1][1] = API_SW_FCU_CPT_WPT;
	API_data[1][2] = API_SW_FCU_CPT_VORDME;
	API_data[1][3] = API_SW_FCU_CPT_NDB;
	API_data[1][4] = API_SW_FCU_CPT_ARPT;
	API_data[1][5] = API_SW_FCU_CPT_RANGE_80;
	API_data[1][6] = API_SW_FCU_CPT_RANGE_40;
	API_data[1][7] = API_SW_FCU_CPT_RANGE_20;

	API_data[2][0] = API_SW_FCU_CPT_MODE_ILS;
	API_data[2][1] = API_SW_FCU_CPT_VOR1;
	API_data[2][2] = API_SW_FCU_CPT_ADF1;
	API_data[2][3] = API_SW_FCU_CPT_LS;
	API_data[2][4] = API_SW_FCU_CPT_FD;
	API_data[2][5] = API_SW_FCU_CPT_QNHinc;
	API_data[2][6] = API_SW_FCU_CPT_QNHdec;
	API_data[2][7] = API_SW_FCU_CPT_QNHpush;

	API_data[3][4] = API_SW_FCU_CPT_mb;
	API_data[3][5] = API_SW_FCU_CPT_inHg;
	API_data[3][7] = API_SW_FCU_CPT_QNHpull;

	API_data[4][0] = API_SW_FCU_SPDinc;
	API_data[4][1] = API_SW_FCU_SPDdec;
	API_data[4][2] = API_SW_FCU_SPDpush;
	API_data[4][3] = API_SW_FCU_SPDpull;
	API_data[4][4] = API_SW_FCU_HDGinc;
	API_data[4][5] = API_SW_FCU_HDGdec;
	API_data[4][6] = API_SW_FCU_HDGpush;
	API_data[4][7] = API_SW_FCU_HDGpull;

	API_data[5][1] = API_SW_FCU_TRK_FPA;
	API_data[5][2] = API_SW_FCU_ATHR;
	API_data[5][3] = API_SW_FCU_LOC;
	API_data[5][4] = API_SW_FCU_AP1;
	API_data[5][5] = API_SW_FCU_AP2;
	API_data[5][6] = API_SW_FCU_CPT_VOR2;
	API_data[5][7] = API_SW_FCU_CPT_ADF2;

	API_data[6][1] = API_SW_FCU_METRICALT;
	API_data[6][2] = API_SW_FCU_ALT1000;
	API_data[6][3] = API_SW_FCU_ALT100;
	API_data[6][4] = API_SW_FCU_VSinc;
	API_data[6][5] = API_SW_FCU_VSdec;
	API_data[6][6] = API_SW_FCU_VSpush;
	API_data[6][7] = API_SW_FCU_VSpull;

	API_data[7][0] = API_SW_FCU_APPR;
	API_data[7][1] = API_SW_FCU_EXPED;
	API_data[7][2] = API_SW_FCU_FO_ADF1;
	API_data[7][3] = API_SW_FCU_FO_VOR1;
	API_data[7][4] = API_SW_FCU_ALTinc;
	API_data[7][5] = API_SW_FCU_ALTdec;
	API_data[7][6] = API_SW_FCU_ALTpull;
	API_data[7][7] = API_SW_FCU_ALTpush;

	API_data[8][0] = API_SW_FCU_FO_MODE_PLAN;
	API_data[8][1] = API_SW_FCU_FO_MODE_ILS;
	API_data[8][2] = API_SW_FCU_FO_RANGE_10;
	API_data[8][3] = API_SW_FCU_FO_RANGE_320;
	API_data[8][4] = API_SW_FCU_FO_RANGE_160;
	API_data[8][5] = API_SW_FCU_FO_RANGE_20;
	API_data[8][6] = API_SW_FCU_FO_RANGE_80;
	API_data[8][7] = API_SW_FCU_FO_RANGE_40;

	API_data[9][0] = API_SW_FCU_FO_ARPT;
	API_data[9][1] = API_SW_FCU_FO_NDB;
	API_data[9][2] = API_SW_FCU_FO_VORDME;
	API_data[9][3] = API_SW_FCU_FO_WPT;
	API_data[9][4] = API_SW_FCU_FO_CSTR;
	API_data[9][5] = API_SW_FCU_FO_MODE_ARC;
	API_data[9][6] = API_SW_FCU_FO_MODE_NAV;
	API_data[9][7] = API_SW_FCU_FO_MODE_VOR;

	API_data[10][1] = API_SW_FCU_FO_QNHpush;
	API_data[10][2] = API_SW_FCU_FO_QNHinc;
	API_data[10][3] = API_SW_FCU_FO_QNHdec;
	API_data[10][4] = API_SW_FCU_FO_FD;
	API_data[10][5] = API_SW_FCU_FO_LS;

	API_data[11][1] = API_SW_FCU_FO_ADF2;
	API_data[11][2] = API_SW_FCU_FO_VOR2;
	API_data[11][4] = API_SW_FCU_FO_mb;
	API_data[11][5] = API_SW_FCU_FO_inHg;
	API_data[11][7] = API_SW_FCU_FO_QNHpull;

}

/**
 * Enregistre l'état des switchs du FCU dans la bank passée en paramètre
 */
void FCU_State_Init(uint8_t * bank)
{
	bank[0] = FCU_ReceiveSW(0, RGPIOA);
	bank[1] = FCU_ReceiveSW(0, RGPIOB);
	bank[2] = FCU_ReceiveSW(1, RGPIOA);
	bank[3] = FCU_ReceiveSW(1, RGPIOB);
	bank[4] = FCU_ReceiveSW(2, RGPIOA);
	bank[5] = FCU_ReceiveSW(2, RGPIOB);
	bank[6] = FCU_ReceiveSW(3, RGPIOA);
	bank[7] = FCU_ReceiveSW(3, RGPIOB);
	bank[8] = FCU_ReceiveSW(4, RGPIOA);
	bank[9] = FCU_ReceiveSW(4, RGPIOB);
	bank[10] = FCU_ReceiveSW(5, RGPIOA);
	bank[11] = FCU_ReceiveSW(5, RGPIOB);
}

/**
 * Vérifie si une modification a été faite sur les switchs du FCU depuis la dernière vérifiaction
 */
int FCU_Check_Changes()
{
	int change = 1;
	for(int i = 0; i < 12; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			if(bitRead(FCU_state[i], j) != bitRead(FCU_state_temp[i], j) )
			{
				diff[i] = bitSet(diff[i], j);
				change = 0;
			}
		}
	}
	return change;
}

/**
 * Envoie la valeur attendue par le simulateur sur le port Série si il y a eu une modification des switchs
 */
void FCU_Transmit_To_Sim()
{
	if (FCU_Check_Changes(FCU_state, FCU_state_temp, diff) == 0)
		  {
			  for(int i = 0; i < 12; i++)
				{
					for(int j = 0; j < 8; j++)
					{
						if(bitRead(diff[i], j) == 1)
						{
							diff[i] = bitClear(diff[i], j);
							if((i == 2 && (j == 6 || j == 5)) || (i == 4 && (j == 5 || j == 4 || j == 1 || j == 0)) ||
							(i == 6 && (j == 5 || j == 4)) || (i == 7 && (j == 5 || j == 4)) || (i == 10 && (j == 3 || j == 2)))
							{
								FCU_Switch_direction(i, j);
							}
							else if(bitRead(FCU_state_temp[i], j) == 0)
							{
								sprintf(buffer,"%d\n", API_data[i][j]);
								Serial_Transmit_Str(buffer);
							}
						}
					}
				}
			  memcpy(FCU_state, FCU_state_temp, sizeof(FCU_state_temp));
		  }
}

/**
 * Détermine la direction d'un switch rotatif et envoie la valeur correcte sur le Port Série
 */
void FCU_Switch_direction(int i, int j)
{

	int k = ((i == 2 && j == 6) || (i == 4 && (j == 5 || j == 1)) || (i == 6 && j == 5) || (i == 7 && j == 5) || (i == 10 && j == 3) ? j-1:
			((i == 2 && j == 5) || (i == 4 && (j == 4 || j == 0)) || (i == 6 && j == 4) || (i == 7 && j == 4) || (i == 10 && j == 2)) ? j : 8);
	if(k == 8)
		return;
	if(k == j)
		j = k+1;

	switch(i)
	{
		case 2:
			SW1 = &SWS[0];
			SW2 = &SWS[1];
			turnedRight = &turnedSW[0];
			turnedLeft = &turnedSW[1];
			break;
		case 4:
			if(j == 5 || j == 4)
			{
				SW1 = &SWS[2];
				SW2 = &SWS[3];
				turnedRight = &turnedSW[2];
				turnedLeft = &turnedSW[3];
			}
			else
			{
				SW1 = &SWS[4];
				SW2 = &SWS[5];
				turnedRight = &turnedSW[4];
				turnedLeft = &turnedSW[5];
			}
			break;
		case 6:
			SW1 = &SWS[6];
			SW2 = &SWS[7];
			turnedRight = &turnedSW[6];
			turnedLeft = &turnedSW[7];
			break;
		case 7:
			SW1 = &SWS[8];
			SW2 = &SWS[9];
			turnedRight = &turnedSW[8];
			turnedLeft = &turnedSW[9];
			break;
		case 10:
			SW1 = &SWS[10];
			SW2 = &SWS[11];
			turnedRight = &turnedSW[10];
			turnedLeft = &turnedSW[11];
			break;
		default:
			return;
	}
	temp_SW1 = bitRead(FCU_state_temp[i], j);
	temp_SW2 = bitRead(FCU_state_temp[i], k);

	  if(temp_SW1 != temp_SW2) // Si on détecte un début de rotation
	  {
		  if(temp_SW1 != *SW1) // Si il est vers la droite
		  {
			  *SW1 = temp_SW1;
			  *turnedRight = 1;
		  }
		  else if(temp_SW2 != *SW2) // Si il est vers la gauche
		  {
			  *SW2 = temp_SW2;
			  *turnedLeft = 1;
		  }
	  }
	  else if(*SW1 != *SW2 && (*turnedRight == 1 || *turnedLeft == 1)) // Quand la rotation est complète
	  {
		  if (*turnedRight == 1 && *SW1 == temp_SW1) // Si on a commencé et continué la rotation vers la droite
		  {
			  sprintf(buffer,"%d\n", API_data[i][k]);
			  Serial_Transmit_Str(buffer);
			  *SW2 = temp_SW2;
			  if(temp_SW2 == 0) FCU_state[i] = bitClear(FCU_state_temp[i], k);
			  else FCU_state[i] = bitSet(FCU_state_temp[i], k);
			  *turnedRight = 0;
		  }
		  else if (*turnedLeft == 1 && *SW2 == temp_SW2) // Si on a commencé et continué la rotation vers la gauche
		  {
			  sprintf(buffer,"%d\n", API_data[i][j]);
			  Serial_Transmit_Str(buffer);
			  *SW1 = temp_SW1;
			  if(temp_SW1 == 0) FCU_state[i] = bitClear(FCU_state_temp[i], k);
			  else FCU_state[i] = bitSet(FCU_state_temp[i], k);
			  *turnedLeft = 0;
		  }
		  else
		  {
			  *SW2 = temp_SW2;
			  *SW1 = temp_SW1;
			  *turnedRight = 0;
			  *turnedLeft = 0;
		  }
	  }

}
