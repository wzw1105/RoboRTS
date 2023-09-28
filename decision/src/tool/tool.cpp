/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
#include "tool/tool.h"


/**
 * @brief 计算一个三维点的欧拉距离
 * 
 * @param x x坐标
 * @param y y坐标
 * @param z z坐标
 * @return double 
 */
double Compute_Distance_Point(double x, double y, double z) {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

/**
 * @brief 计算一个二维点的欧拉距离--距离原点
 * 
 * @param x x坐标
 * @param y y坐标
 * @return double 
 */
double Compute_Distance_Point(double x, double y){
    return sqrt(pow(x, 2) + pow(y, 2));
}

/**
 * @brief 计算两个三维点的欧拉距离
 * 
 * @param x1 
 * @param y1 
 * @param z1 
 * @param x2 
 * @param y2 
 * @param z2 
 * @return double 
 */
double Compute_Distance_BetweenBoth(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2) + pow((z1 - z2), 2));
}


/**
 * @brief 计算两个二维点的欧拉距离
 * 
 * @param x1 
 * @param y1 
 * @param x2 
 * @param y2 
 * @return double 
 */
double Compute_Distance_BetweenBoth(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}


/**
 * @brief 根据已知条件计算发射子弹数量
 * 
 * @param shoot_heat 枪口温度（热量值
 * @param enemy_distance 敌方距离
 * @param area 选择打击的装甲板面积
 * @return uint8_t 
 */
uint8_t Compute_Shoot_Number(uint16_t shoot_heat, double enemy_distance, int area)
{
    uint8_t number = 5;
    number = number + (150 - shoot_heat) / 20;
    return number;
}