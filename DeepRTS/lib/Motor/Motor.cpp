#include "Arduino.h"
#include "Servo.h"
#include "Motor.h"

const unsigned int T100LUT[POWER_MAX-POWER_MIN+1] = {
1100,1101,1103,1104,1106,1107,1108,1110,1112,1115,
1118,1121,1124,1128,1131,1133,1135,1137,1139,1142,
1146,1150,1153,1155,1157,1160,1162,1165,1168,1171,
1174,1178,1181,1185,1189,1192,1195,1198,1201,1204,
1207,1210,1213,1217,1220,1223,1226,1228,1231,1234,
1236,1238,1241,1244,1246,1249,1254,1258,1262,1266,
1269,1274,1279,1283,1287,1291,1295,1300,1303,1307,
1310,1313,1316,1319,1323,1327,1331,1338,1343,1349,
1354,1358,1362,1365,1368,1372,1376,1381,1386,1392,
1397,1403,1410,1415,1420,1425,1431,1439,1448,1458,
1500,
1536,1545,1553,1559,1565,1571,1576,1581,1585,1590,
1594,1599,1603,1607,1612,1615,1619,1624,1629,1633,
1637,1641,1645,1649,1652,1656,1659,1664,1669,1673,
1676,1680,1683,1687,1690,1693,1696,1699,1702,1705,
1707,1710,1713,1717,1720,1725,1729,1733,1737,1741,
1746,1751,1754,1757,1760,1762,1765,1767,1770,1772,
1775,1778,1781,1784,1788,1791,1794,1798,1801,1804,
1806,1809,1811,1813,1815,1817,1819,1821,1824,1826,
1829,1834,1838,1842,1845,1848,1851,1854,1857,1860,
1865,1869,1872,1875,1877,1880,1884,1887,1892,1900};

const unsigned int motor_pins[NUM_MOTORS] = {5, 6, 9, 10, 11};
const bool ccw_motors[NUM_MOTORS] = {false, true, false, true, true};
Servo motors[NUM_MOTORS];
int motor_power[NUM_MOTORS];


// TODO: Maybe needed for BFM
void init_LUT() {

}

void set_motors() {
  double df = calc_droop();
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] *= df;
    int power_sat = MAX(MIN(motor_power[i],POWER_MAX),POWER_MIN);
    if (ccw_motors[i]) {
      power_sat = -power_sat;
    }
    unsigned int lut_idx = power_sat - POWER_MIN;
    // Serial.print(T100LUT[lut_idx]);
    // Serial.print(',');
    motors[i].writeMicroseconds(T100LUT[lut_idx]);
  }
  // Serial.println();
}

void set_motors_raw(long *pwms) {
  if(sizeof(pwms)/sizeof(long) != NUM_MOTORS) return;
  for(int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(pwms[i]);
  }
}

void init_motors() {
  init_LUT();
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(motor_pins[i]);
  }
  // Stop Motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] = 0;
  }
  set_motors();
}

double calc_droop() {
  double voltage = (double)analogRead(DROOP_PIN) * 2.56 * 12.0 / (2.5 * 1023.0);
  Serial.print("Voltage is: "); Serial.println(voltage);
  double droop_factor = 1;
  if (voltage < 9) droop_factor = exp(-2.5*(9-voltage));
  return droop_factor;
}