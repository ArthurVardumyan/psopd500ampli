
/* ��������� �������� ������.
   ���������� ��������:
    500
    1000
*/
#define ODR 500

//����� ����������
#define ELECTRODESNUMBER        8

/* ��������� ����������� */
#define RED_LED_DIS HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define RED_LED_EN HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
#define RED_LED_TOG HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15)

#define GREEN_LED_DIS HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define GREEN_LED_EN HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define GREEN_LED_TOG HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14)

#define BLUE_LED_DIS HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define BLUE_LED_EN HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define BLUE_LED_TOG HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

/* ������� ������ ��� */
#define AD7779_RESET_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define AD7779_RESET_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)

/* ������� ���������-���������� ��� */
#define AD7779_PWR_ENABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define AD7779_PWR_DISABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

/* ������� ���������-���������� ����� */
#define AD7779_VIBRO_ENABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AD7779_VIBRO_DISABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

/* ����������� ��� ��������� ���������� SPI ��� ����� � ��� */
#define AD7779_SPI hspi1
/* ����������� ��� ��������� ���������� UART ��� ����� � PC */
#define AD7779_UART huart2

