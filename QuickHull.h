
#ifndef QUICKHULLNEW_QUICKHULL_H
#define QUICKHULLNEW_QUICKHULL_H

#include <vector>
#include <set>

#define coord std::pair<float, float>

class QuickHull {
public:
    QuickHull(const float xCoords[], const float yCoords[], int numberOfCoords);

    //std::vector<coord> convexHull();
    //std::set<coord> convexHull();
    void convexHull();

private:
    std::vector<coord> inputCoords;
    std::set<coord> convexHullCoordSet;
    std::vector<coord> convexHullCoordVector;

    static bool isAboveLine(coord linePoint1, coord linePoint2, coord targetPoint);

    static float distanceFromLine(coord linePoint1, coord linePoint2, coord targetPoint);

    static float areaOfTriangle(coord point1, coord point2,
                                coord point3);

    static void removeInteriorPoints(coord linePoint1, coord linePoint2, coord targetPoint, std::vector<coord> *subset);

    void quickHull(std::vector<coord> *inputCoords, coord *linePoint1, coord *linePoint2, bool aboveLine);
};


#endif //QUICKHULLNEW_QUICKHULL_H
