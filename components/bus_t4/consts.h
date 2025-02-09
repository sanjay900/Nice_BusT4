#pragma once
/* Type of packet message
   Currently, we are only interested in CMD and INF
   The rest have not been studied in depth, and their numbers have not been verified
   6th byte of CMD and INF packets
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* Number verified, sending commands to automation */
//  LSC = 0x02,  /* Working with scenario lists */
//  LST = 0x03,  /* Working with automation lists */
//  POS = 0x04,  /* request and change the position of automation */
//  GRP = 0x05,  /* sending commands to a group of automation with a specified motor bitmask */
//  SCN = 0x06,  /* working with scenarios */
//  GRC = 0x07,  /* sending commands to a group of automation created through the Nice Screen Configuration Tool */
  INF = 0x08,  /* returns or sets information about the device */
//  LGR = 0x09,  /* working with group lists */
//  CGR = 0x0A,  /* working with group categories created through the Nice Screen Configuration Tool */
};

/*
command menu in the oview hierarchy
9-й байт пакетов CMD
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* используется в ответах STA*/
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // закончен маневр по таймауту
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // частичное открывание
};

/* Ошибки */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // Нет ошибок
  FD = 0xFD,    // Нет команды для этого устройства
  };

// Типы моторов
enum motor_type  : uint8_t {
  SLIDING = 0x01,
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over подъемно-поворотные ворота
  };

//  девятый байт
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* пакет для/от всех */
  FOR_CU = 0x04,  /* пакет для/от блока управления */
  FOR_OXI = 0x0A,  /* пакет для/от приемника OXI */
  };
	
// десятый байт GET/SET пакетов EVT, для пакетов CMD встречалось только значение RUN
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Запрос типа привода */
  INF_STATUS     = 0x01, //	Состояние ворот (Открыто/Закрыто/Остановлено)	
  WHO	         = 0x04,  /* Кто в сети?     */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // manufacturer.
  PRD            = 0x09,   // product.
  INF_SUPPORT    = 0x10, //  Доступные INF команды
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // текущее условное положение автоматики, DPRO924 после этого ждет выставления положений
  MAX_OPN        = 0x12,   // Максимально возможное открывание по энкодеру.
  POS_MAX        = 0x18,   // Максимальное положение (открывания) по энкодеру
  POS_MIN        = 0x19,   // Минимальное положение (закрывания) по энкодеру	
  INF_P_OPN1     = 0x21, //	Частичное открывание1 
  INF_P_OPN2     = 0x22, //	Частичное открывание2
  INF_P_OPN3     = 0x23, //	Частичное открывание3
  INF_SLOW_OPN   = 0x24, // Замедление в открывании
  INF_SLOW_CLS   = 0x25, // Замедление в закрывании	
  OPN_OFFSET     = 0x28, /* Задержка открывания  open offset */
  CLS_OFFSET     = 0x29, /* Задержка закрывания  close offset */
  OPN_DIS        = 0x2a, /* Основные параметры - Разгрузка открытия Open discharge */
  CLS_DIS        = 0x2b, /* Основные параметры - Разгрузка закрытия Close discharge */
  REV_TIME       = 0x31, /* Основные параметры - Длительность реверса (Brief inversion value) */
  OPN_PWR        = 0x4A,    /* Основные параметры - Управление усилием - Усилие открывания */	  	  	  	  	  
  CLS_PWR        = 0x4B,    /* Основные параметры - Управление усилием - Усилие закрывания */	  	  	  	  	  	  
  SPEED_OPN      = 0x42,    /* Основные параметры - Настройка скорости - Скорость открывания */	  	  	  	  	  	  	  
  SPEED_CLS      = 0x43,    /* Основные параметры - Настройка скорости - Скорость закрывания */	  
  SPEED_SLW_OPN  = 0x45,    /* Основные параметры - Настройка скорости - Скорость замедленного открывания */	
  SPEED_SLW_CLS  = 0x46,    /* Основные параметры - Настройка скорости - Скорость замедленного закрывания */	
  OUT1           = 0x51,  /* Настройка выходов */	  
  OUT2           = 0x52,  /* Настройка выходов */	  	  
  LOCK_TIME      = 0x5A,  /* Настройка выходов - Время работы замка */
  S_CUP_TIME     = 0x5C,  /* Настройка выходов - Время работы присоски Suction Cup Time*/	  
  LAMP_TIME      = 0x5B,  /* Настройка выходов - Время работы лампы освещения courtesy light Time*/
  COMM_SBS       = 0x61,  /* Настройка команд - Пошагово */	  
  COMM_POPN      = 0x62,  /* Настройка команд - Открыть частично */	  	  
  COMM_OPN       = 0x63,  /* Настройка команд - Открыть */	  	  	  
  COMM_CLS       = 0x64,  /* Настройка команд - Закрыть */	  
  COMM_STP       = 0x65,  /* Настройка команд - СТОП */		  
  COMM_PHOTO     = 0x68,  /* Настройка команд - Фото */		  
  COMM_PHOTO2    = 0x69,  /* Настройка команд - Фото2 */
  COMM_PHOTO3    = 0x6A,  /* Настройка команд - Фото3 */
  COMM_OPN_STP   = 0x6B,  /* Настройка команд - Стоп при открывании */	  
  COMM_CLS_STP   = 0x6C,  /* Настройка команд - Стоп при закрывании */	 
  IN1            = 0x71,  /* Настройка входов */
  IN2            = 0x72,  /* Настройка входов */
  IN3            = 0x73,  /* Настройка входов */
  IN4            = 0x74,  /* Настройка входов */
  COMM_LET_OPN   = 0x78,  /* Настройка команд - Помеха открыванию */	  	  	  
  COMM_LET_CLS   = 0x79,  /* Настройка команд - Помеха закрыванию */	  	  	  	  

  AUTOCLS        = 0x80,    /* Основные параметры - Автозакрывание */
  P_TIME         = 0x81,    /* Основные параметры - Время паузы */
  PH_CLS_ON      = 0x84,    /* Основные параметры - Закрыть после Фото - Активно */	  
  PH_CLS_VAR     = 0x86,    /* Основные параметры - Закрыть после Фото - Режим */	  	  
  PH_CLS_TIME    = 0x85,    /* Основные параметры - Закрыть после Фото - Время ожидания */	  	  	  
  ALW_CLS_ON     = 0x88,    /* Основные параметры - Всегда закрывать - Активно */	  	  
  ALW_CLS_VAR    = 0x8A,    /* Основные параметры - Всегда закрывать - Режим */	  
  ALW_CLS_TIME   = 0x89,    /* Основные параметры - Всегда закрывать - Время ожидания */	  	  	  
  STAND_BY_ACT   = 0x8c,    /* Основные параметры - Режим ожидания - Активно  ON / OFF */
  WAIT_TIME      = 0x8d,    /* Основные параметры - Режим ожидания - Время ожидания */
  STAND_BY_MODE  = 0x8e,    /* Основные параметры - Режим ожидания - Режим -  safety = 0x00, bluebus=0x01, all=0x02*/
  START_ON       = 0x90,    /* Основные параметры - Настройка пуска - Активно */		  	  
  START_TIME     = 0x91,    /* Основные параметры - Настройка пуска - Время пуска */		  	  	  
  SLOW_ON        = 0xA2,    /* Основные параметры - Замедление */	
  DIS_VAL        = 0xA4,    /* Положение - Значение недопустимо disable value */

  BLINK_ON       = 0x94,    /* Основные параметры - Предмерцание - Активно */		  	  	  	  
  BLINK_OPN_TIME = 0x95,    /* Основные параметры - Предмерцание - Время при открывании */		  	  	  	  	  
  BLINK_CLS_TIME = 0x99,    /* Основные параметры - Предмерцание - Время при закрывании */
  OP_BLOCK       = 0x9a,    /* Основные параметры - Блокирование мотора (Operator block)*/
  KEY_LOCK       = 0x9c,    /* Основные параметры - Блокирование кнопок */
  T_VAL          = 0xB1,    /*Alarm threshold value Порог до обслуживания в количестве маневров*/
  P_COUNT        = 0xB2,    /* Partial count Выделенный счетчик*/
  C_MAIN         = 0xB4,    /* Cancel maintenance Отмена обслуживания */
  DIAG_BB        = 0xD0,     /*   DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /*	состояние входов-выходов	*/
  DIAG_PAR       = 0xD2,    /*  DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // Текущий Маневр
  SUBMNU  = 0x04,  // Подменю
  STA = 0xC0,   // статус в движении
  MAIN_SET = 0x80,   // Основные параметры
  RUN = 0x82,   // Команда для выполнения

  };

	
/* run cmd 11-й байт EVT пакетов */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* запрос на изменение параметров */
  GET = 0x99,   /* запрос на получение параметров */
  GET_SUPP_CMD = 0x89, /* получить поддерживаемые команды */
  };
	
	
	
/* Информация для лучшего понимания состава пакетов в протоколе */
// тело пакета запроса CMD
// пакеты с размером тела 0x0c=12 байт 
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), для команд = 0x0c
  uint8_t for_series;           // серия кому пакет ff = всем
  uint8_t for_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, 1 = CMD, 8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t cmd_mnu;                // Меню команды. cmd_mnu = 1 для команд управления
  uint8_t setup_submnu;            // Подменю, в сочетании с группой команды определяет тип отправляемого сообщения
  uint8_t control_cmd;            // Команда, которая должна быть выполнена
  uint8_t offset;            //  Смещение для ответов. Влияет на запросы вроде списка поддерживаемых комманд
  uint8_t crc2;            // crc2, XOR четырех предыдущих байт
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), для команд = 0x0c

};





// тело пакета ответа RSP
// пакеты с размером тела 0x0e=14 байт 
struct packet_rsp_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e
  uint8_t to_series;           // серия кому пакет ff = всем
  uint8_t to_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, для этих пакетов всегда  8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t cmd_mnu;                // Меню команды. cmd_mnu = 1 для команд управления
  uint8_t sub_inf_cmd;            // Из какого подменю получил команду. Значение меньше на 0x80, чем первоначальное подменю
  uint8_t sub_run_cmd;            // Какую команду получил. Значение больше на 0x80, чем полученная команда
  uint8_t hb_data;             // данные, старший бит
  uint8_t lb_data;            // данные, младший бит
  uint8_t err;               // Ошибки
  uint8_t crc2;            // crc2, XOR четырех предыдущих байт
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e

};
	
 // тело пакета ответа с данными EVT
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e
  uint8_t to_series;           // серия кому пакет ff = всем
  uint8_t to_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, для этих пакетов всегда  8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t whose;                // Чей пакет. Варианты: 00 - общий, 04 - контроллера привода, 0A - приемника OXI
  uint8_t setup_submnu;            // Из какого подменю получил команду. Значение равно первоначальному подменю
  uint8_t sub_run_cmd;            // На какую команду отвечаем. Значение меньше на 0x80, чем отправленная ранее команда
  uint8_t next_data;            // Следующий блок данных
  uint8_t err;               // Ошибки
  uint8_t data_blk;            // Блок данных, может занимать несколько байт
  uint8_t crc2;            // crc2, XOR всех предыдущих байт до девятого (Чей пакет)
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };