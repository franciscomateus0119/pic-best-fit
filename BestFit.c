/*
 * @(#)bestfit.c
 * Last Version: 1.00 23/11/2018
 * Authors: Francisco Mateus Rocha Filho & Michael Alves
 * Federal University of Ceará - IFCE, Fortaleza - Ceará.
 * Todos os Direitos Reservados
 * ---------------------------------------------------
 * Copyright - Este trabalho é protegido pelas leis de direitos autorais contidas na
 * constituição brasileira. É proibido qualquer tipo de venda deste material. As
 * informações contidas neste trabalho podem ser utilizadas para fins de estudo,
 * desde que previamente solicitada a autorização dos autores, desde que dentro
 * dessas leis.
 * 
 * Questões,comentários e solicitações podem ser enviadas ao email:
 * franciscomateus0119@gmail.com
 * ---------------------------------------------------
 * Bestfit Algorithm implementado em um PIC16F18875.
 * 
 * Baseado em conceitos de gerenciamento de memória em sistemas operacionais,
 * o algoritmo bestfit consiste em realizar a alocação de processos(objetos) em
 * blocos(containers) da maneira que o fragmento gerado por esta alocação seja o
 * menor possível. Desta forma, o processo é alocado no menor espaço que o caiba.
 * 
 * Este algoritmo foi desenvolvido como trabalho para a cadeira de Sistemas Embarcados,
 * constituída no curso de Bacharelado em Engenharia de Computação. 
 * ---------------------------------------------------
 * Entrada e Saída do algoritmo Bestfit para o PIC16F18875
 * Entrada: 1500 tamanhos de blocos e 1500 tamanhos de processos.
 * Saída: Fragmento gerado pela alocação dos processos no blocos, se realizada.
 *        Mensagem indicando a não alocação do processo e o motivo, caso contrário.
 * ---------------------------------------------------
 * --  #1.
 * --  Date: Nov,16,2018
 * --  Author: Francisco Mateus Rocha Filho
 * --  Motivo: 
 *     a) Implementação dos pragmas, dos includes e dos defines.
 *     b) Implementação das variáveis.
 *     c) Implementação da função Bestfit().
 *     d) Configuração da função main().
 * ---------------------------------------------------
 * --  #2.
 * --  Date: Nov,19,2018
 * --  Author: Michael Alves
 * --  Motivo: Implementação de vetores constantes
 * ---------------------------------------------------
 * --  #3.
 * --  Date: Nov,21,2018
 * --  Author: Francisco Mateus Rocha Filho
 * --  Motivo: 
 *     a) Implementação de mais variáveis
 *     b) Modificação da função BestFit().
 *     c) Modificação da função main().
 * ---------------------------------------------------
 * --  #4.
 * --  Date: Nov,21,2018
 * --  Author: Michael Alves
 * --  Motivo: Implementação das funções UART_iniciar(), UART_Escrever()
 * --  UART_Escrever_Texto().
 * ---------------------------------------------------
 * --  #5.
 * --  Date: Nov,23,2018
 * --  Author: Francisco Mateus Rocha Filho
 * --  Motivo: Implementação do Bubblesort dentro da função BestFit()
 * ---------------------------------------------------
 * --  #6.
 * --  Date: Nov,23,2018
 * --  Author: Francisco Mateus Rocha Filho
 * --  Motivo: Implementação do laço while(1) dentro da função main().
 * ---------------------------------------------------
 */

/*Por: Francisco Mateus Rocha Filho #1.a)
 * Estou introduzindo as configurações de #pragma, #include e #define, visando
 * o bom funcionamento do PIC16F18875.
 */
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 32000000
#define F_CPU 32000000/64//#define Baud_value(baud_rate) (((float)(F_CPU)/(float)baud_rate)-1)
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)//calculus for UART serial tramission rate

//Fim Francisco Mateus Rocha Filho #1.a)

//--------------------------------------------------------------------------------
//--Seção das Variáveis
//--------------------------------------------------------------------------------

/*Por: Francisco Mateus Rocha Filho #1.b)
 * Estou implementando variáveis que serão utilizadas para o funcionamento do
 * algoritmo bestfit.
 */
int i;//Variável auxiliar/contador
int j;//Variável auxiliar/contador
int a;//Variável auxiliar/contador
int b;//Variável auxiliar/contador
int x=1;//Variável auxiliar/contador
int contador=0;//Variável auxiliar/contador
int aux;//Variável auxiliar/contador
int index1=0;//Variável auxiliar/contador
int indexbloco=0;//Variável auxiliar/contador
int indexprocesso=0; //Variável auxiliar/contador
int temp; //Variável a receber o fragmento gerado da alocação dos processos nos blocos

/*Por: Michael Alves #2
 * Estou implementando variáveis constantes que armazenarão os valores de entrada.
 * Estes valores estão armazenados na ROM e não poderão ser modificados.
 */
//Blocos = PA de 1 iniciando com 2 || Processos = PA de 1 iniciando com 1
const int vetorbloco[1500] = { 2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031,1032,1033,1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047,1048,1049,1050,1051,1052,1053,1054,1055,1056,1057,1058,1059,1060,1061,1062,1063,1064,1065,1066,1067,1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080,1081,1082,1083,1084,1085,1086,1087,1088,1089,1090,1091,1092,1093,1094,1095,1096,1097,1098,1099,1100,1101,1102,1103,1104,1105,1106,1107,1108,1109,1110,1111,1112,1113,1114,1115,1116,1117,1118,1119,1120,1121,1122,1123,1124,1125,1126,1127,1128,1129,1130,1131,1132,1133,1134,1135,1136,1137,1138,1139,1140,1141,1142,1143,1144,1145,1146,1147,1148,1149,1150,1151,1152,1153,1154,1155,1156,1157,1158,1159,1160,1161,1162,1163,1164,1165,1166,1167,1168,1169,1170,1171,1172,1173,1174,1175,1176,1177,1178,1179,1180,1181,1182,1183,1184,1185,1186,1187,1188,1189,1190,1191,1192,1193,1194,1195,1196,1197,1198,1199,1200,1201,1202,1203,1204,1205,1206,1207,1208,1209,1210,1211,1212,1213,1214,1215,1216,1217,1218,1219,1220,1221,1222,1223,1224,1225,1226,1227,1228,1229,1230,1231,1232,1233,1234,1235,1236,1237,1238,1239,1240,1241,1242,1243,1244,1245,1246,1247,1248,1249,1250,1251,1252,1253,1254,1255,1256,1257,1258,1259,1260,1261,1262,1263,1264,1265,1266,1267,1268,1269,1270,1271,1272,1273,1274,1275,1276,1277,1278,1279,1280,1281,1282,1283,1284,1285,1286,1287,1288,1289,1290,1291,1292,1293,1294,1295,1296,1297,1298,1299,1300,1301,1302,1303,1304,1305,1306,1307,1308,1309,1310,1311,1312,1313,1314,1315,1316,1317,1318,1319,1320,1321,1322,1323,1324,1325,1326,1327,1328,1329,1330,1331,1332,1333,1334,1335,1336,1337,1338,1339,1340,1341,1342,1343,1344,1345,1346,1347,1348,1349,1350,1351,1352,1353,1354,1355,1356,1357,1358,1359,1360,1361,1362,1363,1364,1365,1366,1367,1368,1369,1370,1371,1372,1373,1374,1375,1376,1377,1378,1379,1380,1381,1382,1383,1384,1385,1386,1387,1388,1389,1390,1391,1392,1393,1394,1395,1396,1397,1398,1399,1400,1401,1402,1403,1404,1405,1406,1407,1408,1409,1410,1411,1412,1413,1414,1415,1416,1417,1418,1419,1420,1421,1422,1423,1424,1425,1426,1427,1428,1429,1430,1431,1432,1433,1434,1435,1436,1437,1438,1439,1440,1441,1442,1443,1444,1445,1446,1447,1448,1449,1450,1451,1452,1453,1454,1455,1456,1457,1458,1459,1460,1461,1462,1463,1464,1465,1466,1467,1468,1469,1470,1471,1472,1473,1474,1475,1476,1477,1478,1479,1480,1481,1482,1483,1484,1485,1486,1487,1488,1489,1490,1491,1492,1493,1494,1495,1496,1497,1498,1499,1500,1501 };
//Vetor bloco constituído de 1500 valores (tamanhos de bloco)
const int vetorprocesso[1500] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,932,933,934,935,936,937,938,939,940,941,942,943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031,1032,1033,1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047,1048,1049,1050,1051,1052,1053,1054,1055,1056,1057,1058,1059,1060,1061,1062,1063,1064,1065,1066,1067,1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080,1081,1082,1083,1084,1085,1086,1087,1088,1089,1090,1091,1092,1093,1094,1095,1096,1097,1098,1099,1100,1101,1102,1103,1104,1105,1106,1107,1108,1109,1110,1111,1112,1113,1114,1115,1116,1117,1118,1119,1120,1121,1122,1123,1124,1125,1126,1127,1128,1129,1130,1131,1132,1133,1134,1135,1136,1137,1138,1139,1140,1141,1142,1143,1144,1145,1146,1147,1148,1149,1150,1151,1152,1153,1154,1155,1156,1157,1158,1159,1160,1161,1162,1163,1164,1165,1166,1167,1168,1169,1170,1171,1172,1173,1174,1175,1176,1177,1178,1179,1180,1181,1182,1183,1184,1185,1186,1187,1188,1189,1190,1191,1192,1193,1194,1195,1196,1197,1198,1199,1200,1201,1202,1203,1204,1205,1206,1207,1208,1209,1210,1211,1212,1213,1214,1215,1216,1217,1218,1219,1220,1221,1222,1223,1224,1225,1226,1227,1228,1229,1230,1231,1232,1233,1234,1235,1236,1237,1238,1239,1240,1241,1242,1243,1244,1245,1246,1247,1248,1249,1250,1251,1252,1253,1254,1255,1256,1257,1258,1259,1260,1261,1262,1263,1264,1265,1266,1267,1268,1269,1270,1271,1272,1273,1274,1275,1276,1277,1278,1279,1280,1281,1282,1283,1284,1285,1286,1287,1288,1289,1290,1291,1292,1293,1294,1295,1296,1297,1298,1299,1300,1301,1302,1303,1304,1305,1306,1307,1308,1309,1310,1311,1312,1313,1314,1315,1316,1317,1318,1319,1320,1321,1322,1323,1324,1325,1326,1327,1328,1329,1330,1331,1332,1333,1334,1335,1336,1337,1338,1339,1340,1341,1342,1343,1344,1345,1346,1347,1348,1349,1350,1351,1352,1353,1354,1355,1356,1357,1358,1359,1360,1361,1362,1363,1364,1365,1366,1367,1368,1369,1370,1371,1372,1373,1374,1375,1376,1377,1378,1379,1380,1381,1382,1383,1384,1385,1386,1387,1388,1389,1390,1391,1392,1393,1394,1395,1396,1397,1398,1399,1400,1401,1402,1403,1404,1405,1406,1407,1408,1409,1410,1411,1412,1413,1414,1415,1416,1417,1418,1419,1420,1421,1422,1423,1424,1425,1426,1427,1428,1429,1430,1431,1432,1433,1434,1435,1436,1437,1438,1439,1440,1441,1442,1443,1444,1445,1446,1447,1448,1449,1450,1451,1452,1453,1454,1455,1456,1457,1458,1459,1460,1461,1462,1463,1464,1465,1466,1467,1468,1469,1470,1471,1472,1473,1474,1475,1476,1477,1478,1479,1480,1481,1482,1483,1484,1485,1486,1487,1488,1489,1490,1491,1492,1493,1494,1495,1496,1497,1498,1499,1500 };
//Vetor processo constituído de 1500 valores (tamanhos de processo)

//Fim Michael Alves #2


int processos[100];//Vetor no qual serão armazenados 100 tamanhos de processo, provenientes do vetorprocesso[].
int blocos[100];//Vetor no qual serão armazenados 100 tamanhos de blocos, provenientes do vetorbloco[].
char texto[100];//Vetor no qual será armazenado a mensagem resultado e mostrado pelo serial
//Fim Francisco Mateus Rocha Filho #1.b)

/*Por: Francisco Mateus Rocha Filho #3.a)
 * Estou implementando variáveis devido as mudanças na função Bestfit em #3.b).
 * Será possível ver se os processos foram ou não alocados e, se não, o motivo
 * de isto ter acontecido.
 */

int naoalocadop=0; //Variável contador que informa a quantidade de processos que não foram alocados
int tamanhomax = 1501;//Maior tamanho de bloco dentre os tamanhos

//Fim Francisco Mateus Rocha Filho #3.a)


//--------------------------------------------------------------------------------
//--Seção Protocolo Serial UART
//--------------------------------------------------------------------------------

/*Por: Michael Alves #4
 * Implementação e configuração das funções referentes ao protocolo serial UART.
 * Necessário para a visualização da saída.
 */

//Função para inicializar o protocolo serial. Configuração dos pinos.
void UART_iniciar(){
    TXSTAbits.TXEN = 1;//
    RCSTAbits.SPEN = 1;//Enable serial port
    ANSELCbits.ANSC7 = 0;   //Desativa função analógica no pino RC7
    ANSELCbits.ANSC6 = 0;   //Desativa função analógica no pino RC6
    RC6PPS = 0x10;          //Configura pino RC6 como TX
    RXPPSbits.RXPPS = 0x17; //Configura pino RC7 como RX
    TX1STAbits.BRGH = 0;    //Necessário para configurar o baud rate da EUSART
    BAUDCON1bits.BRG16 = 0; //Configura registrado BAUDCON1 como 8 bits
    SPBRG = 51;             //Configura baud rate para 9600bps
    RC1STAbits.SPEN = 1;    //Habilita EUSART
    RC1STAbits.CREN = 1;    //Habilita recebimento EUSART (RX)
    TX1STAbits.TXEN = 1;    //Habilita transmissão EUSART (TX)
    TX1STAbits.SYNC = 0;    //Configura EUSART para modo assíncrono
}

//Função para escrever uma caractere através do serial.
void UART_Escrever(char c){
    while(!TX1STAbits.TRMT); //Enquanto o pino TX não estiver pronto, aguarde;
    TXREG = c;//Printar o caractere c.
}


//Função para escrever mais de um caractere (strings) através do serial.
void UART_Escrever_Texto(char* texto){
    int index;
    for(index=0; texto[index]!='\0'; index++)//Para cada caractere contido na mensagem a ser enviada
        UART_Escrever(texto[index]);//Mande o caractere localizado em texto[index].
}
//Fim Michael Alves #4.


//--------------------------------------------------------------------------------
//--Seção Aplicação do Algoritmo BestFit
//--------------------------------------------------------------------------------

/*Por: Francisco Mateus Rocha Filho #1.c)
 * Estou implementando a função BestFit(). Ela é responsável pela alocação
 * dos processos nos blocos. Armazena-se 100 tamanhos de processos e 100
 * tamanhos de blocos em 2 vetores diferentes, puxando esses tamanhos dos vetores
 * constantes armazenados na ROM. Escolhe-se, assim, um processo e tenta alocá-lo
 * no bloco de forma que seja gerado o menor fragmento possível. Caso isto ocorra,
 * é dito qual o tamanho do processo, o tamanho do bloco e o fragmento gerado.
 */

//Função que realiza a implementação do algoritmo bestfit no PIC16f18875
void BestFit(void){
    while(index1<15){//index<15 por serão realizadas 15 vezes 100 alocações
		for(a=0;a<100;a++){//puxa 100 valores de vetorbloco(constante) e armazena em blocos
				b=a+indexbloco;//Usado como index para puxar corretamente os valores do vetorbloco[].
	            blocos[a] = vetorbloco[b];     
	        }
	    
	    for(a=0;a<100;a++){//puxa 100 valores de vetorprocesso(constante) e armazena em processos)
	    		b=a+indexprocesso;//Usado como index para puxar corretamente os valores do vetorprocesso[].
	            processos[a] = vetorprocesso[b];            
	        }
	    
	    for(j=0;j<100;j++){//Para cada processo...
	        
	        x=1;//Verifica se o encaixe ou não encaixe foi realizado
	        while(x!=0){
	            
	            for(i=0;i<100;i++){//Percorre todos os blocos do vetor blocos
	                if(processos[j]<=blocos[i] && processos[j]!=0){/*Se Tamanho do processo < Tamanho do Bloco e Bloco =/= 0 */
	                    temp = blocos[i] - processos[j];//Fragmento gerado
	                    contador++;//Numero do encaixe				
                        //Armazena a mensagem resultado na variável global texto
                        sprintf(texto,"%d - T_Processo = %d       T_Bloco = %d       Fragmento = %d\n",contador,processos[j],blocos[i],temp);
	                    UART_Escrever_Texto(texto);//Mostrar o resultado pela serial.
                        processos[j]=0;//Processos que foram encaixados tem eu valor mudado para 0. Para não ter erro de selecioná-los novamente.
	                    blocos[i] = temp;//O bloco tem o seu tamanho atualizado com o que restou depois do encaixe
	                    
                        /*Por: Francisco Mateus Rocha Filho #5
                        * Implementação da estratégia de ordenação Bubblesort.
                        * Responsável por reorganizar o vetor blocos[] após o
                        * mesmo ter seu valor atualizado com o fragmento gerado.
                        */
                        
                        //Organizar Vetor Blocos
	                	for(a=0;a<100;a++){
					        for(b=a+1;b<100;b++){
					            if(blocos[b]<blocos[a]){
					                aux = blocos[b];
					                blocos[b] = blocos[a];
					                blocos[a] = aux;
					            }
					        }
					    }
                        // FIM Francisco Mateus Rocha Filho #5
					    
					}
					
	            }
                
	            /*Por: Francisco Mateus Rocha Filho #3.b)
                * Com estas regras, é possível saber o motivo pelo qual o processo
                * não pôde ser alocado, seja por ser maior que o maior tamanho
                * de bloco dentre todos os blocos ou dentre os 100 blocos que
                * estão sendo utilizados.
                */
	            i--;
	            if(processos[j]>tamanhomax){//Se o tamanho do processo for maior do que o maior tamanho de blocos existnte, ele não poderá ser alocado
                naoalocadop++;//Número de não alocados +1
                //printf("\n%d.N - T_Processo = %d nao alocado - maior espaco = %d\n",naoalocadop, processos[j],tamanhomin);
            	sprintf(texto,"\n%d.N - T_Processo = %d nao alocado - maior espaco = %d\n",naoalocadop, processos[j],tamanhomax);
            	UART_Escrever_Texto(texto);
                }
	            else if(processos[j]>blocos[99]){//Se o processo for maior que o maior tamanho dentre os 100 tamanhos de blocos disponíveis, ele não será alocado
                naoalocadop++;//Número de alocados +1
                sprintf(texto,"\n%d.N - T_Processo = %d nao alocado - maior espaco = %d\n",naoalocadop, processos[j],blocos[i-1]);
            	UART_Escrever_Texto(texto);
                }
                // FIM Francisco Mateus Rocha Filho #3.b)
            	
	            x=0;//Sinaliza o fim da tentativa de encaixe do processo número j
	        }
	        
	    }
	    index1++;//Partir para próxima interação
	    indexbloco+=100;//Puxar os próximos 100 valores de blocos
	    indexprocesso+=100;//Puxar os próximos 100 valores de blocos
	   
	}
}


//Fim Francisco Mateus Rocha Filho #1.c)


//--------------------------------------------------------------------------------
//--Seção Main
//--------------------------------------------------------------------------------

/*Por: Francisco Mateus Rocha Filho #1.d)
 * Configurando a função main para que o algoritmo seja finalizado. A função
 * UART_iniciar() e BestFit são inicializadas.
 */   

int main(void) {
    UART_iniciar(); 
    
    /*Por: Francisco Mateus Rocha Filho #6
    * Implementado o laço infinito para facilitar a observação dos resultados
    * das alocações realizadas pela função BestFit().
    */ 
    while(1){//Laço infinito por motivos de observação de resultado. Pode ser removido, desde que as variáveis não sejam alteradas.
        index1 = 0;//Recebe 0 quando a aplicação do algoritmo é reiniciada. Para fins de observação de resultado.
        indexbloco = 0;//Recebe 0 quando a aplicação do algoritmo é reiniciada. Para fins de observação de resultado.
        indexprocesso = 0;//Recebe 0 quando a aplicação do algoritmo é reiniciada. Para fins de observação de resultado.
        contador = 0;//Recebe 0 quando a aplicação do algoritmo é reiniciada. Para fins de observação de resultado.
        
        /*Por: Francisco Mateus Rocha Filho #3.c)
        * Valor da variável "naoalocadop" visto as mudanças em #3.a) e #3.b).
        */  
        naoalocadop = 0;
        //Fim Francisco Mateus Rocha Filho #3.c)
        
        BestFit();
    }
    
    //Fim Francisco Mateus Rocha Filho #6
       
 
}
//Fim Francisco Mateus Rocha Filho #1.d)
