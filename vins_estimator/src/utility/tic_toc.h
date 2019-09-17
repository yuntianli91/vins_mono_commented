/*
 * @Description: timer tool
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-08-30 09:05:05
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-08-30 09:05:16
 */
#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }
    /**
     * @brief get start time when TicToc construct
     * 
     */
    void tic()
    {
        start = std::chrono::system_clock::now();
    }
    /**
     * @brief return elapsed time in milliseconds
     * 
     * @return 
     *  @rtval double, elapsed time since TicToc constructed in milliseconds 
     */
    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
