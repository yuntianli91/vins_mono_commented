#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
/**
 * @brief 类TicToc中函数：tic()获得计时起点；toc()获得运行时间，单位毫秒。
 * chrono是一个C++标准时间库，为模板库，包括三个概念：duration、time_point、clock。
 * 
 */
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now(); ///获得计时起始点
    }

    double toc()
    {
        end = std::chrono::system_clock::now(); ///获得计时终止点
        std::chrono::duration<double> elapsed_seconds = end - start; ///获得运行时间，默认单位秒
        return elapsed_seconds.count() * 1000; ///duration成员函数count()返回Rep类型的Period的数量，参考https://www.jianshu.com/p/80de04b41c31
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end; ///定义时间起点和终点
};
