
#ifndef QUICKHULLNEW_QUICKHULL_H
#define QUICKHULLNEW_QUICKHULL_H

#include <vector>
#include <set>

#define coord std::pair<float, float>

class QuickHull {
public:
    /** Creates QuickHull object
     *
     * @param xCoords X coordinates of points
     * @param yCoords Y coordinates of points
     * @param numberOfCoords Number of points
     */
    QuickHull(const float xCoords[], const float yCoords[], int numberOfCoords);

    /** \brief Returns convex hull of QuickHull objects x and y coordinates.
     *
     * Splits points by the line that joins the two points most extreme in x values.
     * For each side of this line, finds the point furthest from the line and
     * removes all points enclosed by the triangle made by these three points.
     *
     * Recursively uses the two newly formed sides of the triangle as the next lines and repeats the process above.
     *
     * If no points are beyond any given line is is said to form part of the convex hull.
     *
     * When there are no more points to check all points that form the convex hull are returned.
     *
     * @return [std::set<coord>] pairs of floating point numbers (x,y) that form the convex hull of points.
     */
    std::set<coord> convexHull();

private:
    std::vector<coord> inputCoords;
    std::set<coord> convexHullCoordSet;
    std::vector<coord> convexHullCoordVector;

    /** Checks if point is above a line defined by two other points or not.
     *
     * @param linePoint1 One of the two points that forms the reference line.
     * @param linePoint2 One of the two points that forms the reference line.
     * @param targetPoint The point being tested if it is above the line or not.
     * @return [bool] Boolean to indicated if point is above line or not.
     */
    static bool isAboveLine(coord linePoint1, coord linePoint2, coord targetPoint);

    /** Calculates the distance of a point from a line defined by two other points.
     *
     * @param linePoint1 One of the two points that forms the reference line.
     * @param linePoint2 One of the two points that forms the reference line.
     * @param targetPoint The point that's distance from the line will be returned.
     * @return [float] The distance of the point from the line.
     */
    static float distanceFromLine(coord linePoint1, coord linePoint2, coord targetPoint);

    /** Calculates are of a triangle defined by three points.
     *
     * @param point1 One of the three points that forms the triangle.
     * @param point2 One of the three points that forms the triangle.
     * @param point3 One of the three points that forms the triangle.
     * @return [float] The area of the triangles defined by the three points.
     */
    static float areaOfTriangle(coord point1, coord point2,
                                coord point3);

    /** \brief Checks if points are within triangle and deletes them from subset if they are.
     *
     *
     * For each point in the vector subset points to:
     *      Finds the sum area of the three triangles that can be made using each of the sides of the triangle
     *      made of linePoint1, linePoint2, and target point.
     *
     *      Compares this to the area of the original triangle, if they are equal then this point is contained
     *      within the original triangle.
     *
     *      A integer is emplaced into a vector called flags.
     *      1 to indicated the point is not within the triangle, 0 to indicate it is.
     *
     * The flag vector is iterated through, all items that are 0 have the corresponding, by index, items
     * in the vector that the subset pointer points to removed.
     *
     * @param linePoint1 One of the three points that defines original triangle.
     * @param linePoint2 One of the three points that defines original triangle.
     * @param targetPoint One of the three points that defines original triangle.
     * @param subset Pointer a vector of points to be checked adn altered accordingly.
     */
    static void removeInteriorPoints(coord linePoint1, coord linePoint2, coord targetPoint, std::vector<coord> *subset);

    /** \brief Recursively finds the convex hull of a given subset of points on one side of a line.
     *
     *
     *
     * @param inputCoords The coordinate pairs of points to be checked.
     * @param linePoint1 One of the points that makes up the dividing line.
     * @param linePoint2 One of the points that makes up the dividing line.
     * @param aboveLine If the algorithm is preforming on points above or below the line.
     */
    void quickHull(std::vector<coord> *inputCoords, coord *linePoint1, coord *linePoint2, bool aboveLine);
};


#endif //QUICKHULLNEW_QUICKHULL_H
