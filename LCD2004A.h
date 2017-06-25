/*
 * LCD2004A.h - library for LCD2004A with I2C backpack
 */

// ensure this library description is only included once
#ifndef __LCD2004A_H__
#define __LCD2004A_H__

class LCD2004A {
  public:
    LCD2004A();
    boolean setup();
    void printVoltages(int line, float v1, float v2, float v3);
    void printRaw(int line, int v1, int v2, int v3);
    void test();
  private:
};

#endif
