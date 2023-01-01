
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "QuickHull.h"

/*
 * Not my work!
 * Copied from https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 */
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp)
{
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    return std::fabs(x-y) <= std::numeric_limits<T>::epsilon() * std::fabs(x+y) * ulp
           // unless the result is subnormal
           || std::fabs(x-y) < std::numeric_limits<T>::min();
}

QuickHull::QuickHull(const float *xCoords, const float *yCoords, int numberOfCoords) {

    if (numberOfCoords <= 2) {
        throw std::invalid_argument("At least 3 points are needed to form a convex hull.");
    }

    for (int i = 0; i < numberOfCoords; i++) {
        coord nextCoord = {xCoords[i], yCoords[i]};
        inputCoords.emplace_back(nextCoord);
    }

    std::sort(inputCoords.begin(), inputCoords.end());
}

std::set<coord> QuickHull::convexHull() {
    // add both extreme x points to convex hull
    // (vector is sorted by x values in constructor)
    coord minXPoint = inputCoords[ 0 ] ;
    coord maxXPoint = inputCoords[ inputCoords.size()-1 ];

    convexHullCoordSet.insert(minXPoint);
    convexHullCoordSet.insert(maxXPoint);

    // removes extreme x point from input coords
    inputCoords.erase(inputCoords.begin());
    inputCoords.erase(inputCoords.end());

    // split input coords into two subsets: above and below the line
    std::vector<coord> topSubset;
    std::vector<coord> bottomSubset;

    for (coord i : inputCoords) {
        if (isAboveLine(minXPoint, maxXPoint, i)) {
            topSubset.emplace_back(i);
        } else {
            bottomSubset.emplace_back(i);
        }
    }


    quickHull(&topSubset, &minXPoint, &maxXPoint, isAboveLine(minXPoint, maxXPoint, topSubset[0]));


    quickHull(&bottomSubset, &minXPoint, &maxXPoint, isAboveLine(minXPoint, maxXPoint, bottomSubset[0]));

    std::set<coord> convexHull(convexHullCoordVector.begin(),
                               convexHullCoordVector.end());

    return convexHull;
}

bool QuickHull::isAboveLine(std::pair<float, float> linePoint1, std::pair<float, float> linePoint2,
                            std::pair<float, float> targetPoint) {
    float crossProduct = (targetPoint.second - linePoint1.second) * (linePoint2.first - linePoint1.first) -
            (linePoint2.second - linePoint1.second) * (targetPoint.first - linePoint1.first);
    if (crossProduct > 0) {
        return true;
    } else {
        return false;
    }
}

float QuickHull::distanceFromLine(std::pair<float, float> linePoint1, std::pair<float, float> linePoint2,
                                  std::pair<float, float> targetPoint) {
    float nominator = std::fabs( (targetPoint.first - linePoint1.first) * (linePoint2.second - linePoint1.second) -
                                         (targetPoint.second - linePoint1.second) * (linePoint2.first - linePoint1.first) );
    float denominator = std::sqrt( std::pow( (linePoint2.second - linePoint1.second) , 2.0f) +
                                   std::pow( (linePoint2.first - linePoint1.first), 2.0f )  );

    return (nominator/denominator);
}

float QuickHull::areaOfTriangle(std::pair<float, float> point1, std::pair<float, float> point2,
                                std::pair<float, float> point3) {
    // Area = 0.5[x1(y2 - y3) + x2(y3 - y1) + x3(y1 - y2)]
    return 0.50f*fabsf(( point1.first*(point2.second - point3.second) + point2.first*( point3.second - point1.second ) + point3.first*( point1.second - point2.second) ));
}

void QuickHull::removeInteriorPoints(coord linePoint1, coord linePoint2, coord targetPoint, std::vector<coord> *subset) {
    float originalArea = areaOfTriangle(linePoint1, linePoint2, targetPoint);

    int lengthCompensation = 0;

    std::vector<int> flags;

    for (auto point : *subset) {
        float area = areaOfTriangle(point, linePoint2, targetPoint) + areaOfTriangle(linePoint1, point, targetPoint) +
                     areaOfTriangle(linePoint1, linePoint2, point);

        if (almost_equal(area, originalArea, 3)) {
            flags.emplace_back(0);
        } else {
            flags.emplace_back(1);
        }
    }


    for (int i = 0; i < (int) flags.size(); i++) {
        if (flags.at(i) == 0){
            subset->erase( subset->begin()+i-lengthCompensation);
            lengthCompensation += 1;
        }
    }


}

void QuickHull::quickHull(std::vector<coord> *coordVector, coord *linePoint1,
                          coord *linePoint2, bool aboveLine) {
    bool exteriorPoint = true;
    int maxDistanceIndex = 0;
    float maxDistance = 0;

    for (int i = 0; i < (int) coordVector->size(); i++) {
        float distance = distanceFromLine(*linePoint1, *linePoint2, coordVector->at(i));
        if (distance > maxDistance) {
            maxDistance = distance;
            maxDistanceIndex = i;
            exteriorPoint = false;
        }
    }

    if (exteriorPoint) {
        convexHullCoordVector.emplace_back(*linePoint1);
        convexHullCoordVector.emplace_back(*linePoint2);
        return;
    }

    coord newTargetCoord = coordVector->at(maxDistanceIndex);

    removeInteriorPoints(*linePoint1, *linePoint2, newTargetCoord, coordVector);

    quickHull(coordVector, linePoint1, &newTargetCoord, aboveLine);
    quickHull(coordVector, linePoint2, &newTargetCoord, aboveLine);
}
