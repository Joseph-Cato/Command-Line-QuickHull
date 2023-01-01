
#include <iostream>
#include <chrono>
#include <random>

#include "QuickHull.h"

int main() {
    /*
    *
    *       This file is just to demonstrate the QuickHull class in action and time its performance.
     *      Nothing here affects the algorithm and can all be removed.
    *
    *
    */

    std::cout << "\n-----------------------------QuickHull Demo:-----------------------------\n\n\n";

    float xInput[] = {4.36, -3.2, -10.9, 2.11, 0.6,  -0.4,  -10.07, 8.54, 1, 2, 3, 4.35};
    float yInput[] = {-2.28, 1.8, -3.48, 0.4,  5.32, -5.04, -3.36,  2.9,  1, 2, 4, 4.84};


    QuickHull object = QuickHull(xInput, yInput, 12);

    std::cout << "Calculating Convex hull for points: ";
    for (int i = 0; i < (int) sizeof(xInput)/sizeof(xInput[0])-1; i++) {
        std::cout << "(" << xInput[i] << ", " << yInput[i] << "), ";
    }
    std::cout << "(" << xInput[(sizeof(xInput)/sizeof(xInput[0]))-1] << ", " << yInput[(sizeof(xInput)/sizeof(xInput[0]))-1] << ")\n\n";

    auto start = std::chrono::steady_clock::now();

    std::set<std::pair<float, float>> result = object.convexHull();

    auto stop = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = stop - start;


    std::cout << "Done in " << diff.count() <<"s.\nConvex Hull consists of points:\n";
    for (std::pair<float, float> i : result) {
        std::cout << "( " << i.first << ", " << i.second << "), ";
    }
    std::cout << std::endl;

    int arraySize = 100000;

    std::cout << "\n\n-----------------------------QuickHull Demo 2:---------------------------\n\n\n";

    std::cout << "Generating arrays of " << arraySize << " random floating point elements from (-100,-100) to (100,100)...";

    std::default_random_engine gen;
    gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> distribution(-100.0,100.0f);

    float xInput2[arraySize];
    float yInput2[arraySize];

    for (int i = 0; i < arraySize; i++) {
        xInput2[i] = distribution(gen);
        yInput2[i] = distribution(gen);
    }

    QuickHull object2 = QuickHull(xInput2, yInput2, arraySize);

    std::cout << "\nDone.\n\nCalculating convex hull from these points...\n";

    auto start2 = std::chrono::steady_clock::now();

    std::set<std::pair<float, float>> result2 = object2.convexHull();

    auto stop2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff2 = stop2 - start2;

    std::cout << "\nDone in " << diff2.count() <<"s.\n" << std::endl;

    return 0;
}