
#include <iostream>

#include "QuickHull.h"

int main() {

    float xInput[] = {4.36, -3.2, -10.9, 2.11, 0.6,  -0.4,  -10.07, 8.54, 1, 2, 3, 4.35};
    float yInput[] = {-2.28, 1.8, -3.48, 0.4,  5.32, -5.04, -3.36,  2.9,  1, 2, 4, 4.84};

    QuickHull object = QuickHull(xInput, yInput, 12);
    //std::vector<std::pair<float, float>> convexHull = object.convexHull();
    //std::set<coord> convexHull = object.convexHull();
    object.convexHull();


/*
    for (std::pair<float, float> i : convexHull) {
        std::cout << "( " << i.first << ", " << i.second << "),    ";
    }
    std::cout << std::endl;
    */

    return 0;
}