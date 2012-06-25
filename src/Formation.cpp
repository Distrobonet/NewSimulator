////
//// Filename:        "Formation.cpp"
////
//// Programmer:      Ross Mead
//// Last modified:   30Nov2009
////
//// Description:     This class implements a formation.
////
//
//// preprocessor directives
//#include <Simulator/Formation.h>
//
//
//
//// <constructors>
//
//// Default constructor that initializes
//// this formation to the parameterized values.
//Formation::Formation(const Function f,
//                     const float  r,
//                     const Vector   sGrad,
//                     const int    sID,
//                     const int    fID,
//                     const float  theta)
//{
//    setFunction(f);
//    setRadius(r);
//    setSeedFrp(sGrad);
//    setSeedID(sID);
//    setFormationID(fID);
//    setHeading(theta);
//}
//
//
//// Default constructor that initializes
//// the formation to the parameterized values.
//Formation::Formation(vector<Function> f,
//                     const float    r,
//                     const Vector     sGrad,
//                     const int      sID,
//                     const int      fID,
//                     const float    theta)
//{
//    setFunctions(f);
//    setRadius(r);
//    setSeedFrp(sGrad);
//    setSeedID(sID);
//    setFormationID(fID);
//    setHeading(theta);
//
//
//}
//
//
//// Copy constructor that copies the contents of
//// the parameterized formation into this formation.
//Formation::Formation(const Formation &f)
//{
//    *this = f;
//}
//
//
//// Attempts to set the function to the parameterized function,
//// returning true if successful, false otherwise.
//bool Formation::setFunction(const Function f)
//{
//    clear();
//    return addFunction(f);
//}
//
//
//// Attempts to set the set of functions
//// to the parameterized set of functions,
//// returning true if successful, false otherwise.
//bool Formation::setFunctions(const vector<Function> &f)
//{
//    clear();
//    return addFunctions(f);
//}
//
//
//// Attempts to add the parameterized function to the formation,
//// returning true if successful, false otherwise.
//bool Formation::addFunction(const Function f)
//{
//    if (f == NULL) return false;
//    push_back(f);
//    return true;
//}
//
//
//// Attempts to add the parameterized set of functions to the formation,
//// returning true if successful, false otherwise.
//bool Formation::addFunctions(const vector<Function> &f)
//{
//    for (uint i = 0; i < f.size(); ++i)
//        if (!addFunction(f[i]))
//        	return false;
//    return true;
//}
//
//
//// Attempts to remove the function at the
//// parameterized position from the formation,
//// returning true if successful, false otherwise.
//bool Formation::removeFunction(const int pos)
//{
//    if ((pos < 0) || (pos >= size()))
//    	return false;
//    erase(begin() + pos);
//    return true;
//}
//
//
//// Attempts to remove all of the functions from the formation,
//// returning true if successful, false otherwise.
//bool Formation::removeFunctions()
//{
//    clear();
//    return true;
//}
//
//
//// Attempts to set the radius to the parameterized radius,
//// returning true if successful, false otherwise.
//bool Formation::setRadius(const float r)
//{
//    if (r <= 0.0f)
//    	return false;
//    radius = r;
//    return true;
//}
//
//
//// Attempts to set the seed gradient
//// to the parameterized seed gradient,
//// returning true if successful, false otherwise.
//bool Formation::setSeedFrp(const Vector sGrad)
//{
//	seedFrp = sGrad;
//    return true;
//}
//
//
//// Attempts to set the seed ID to the parameterized seed ID,
//// returning true if successful, false otherwise.
//bool Formation::setSeedID(const int sID)
//{
//    if ((sID < 0) && (sID != ID_OPERATOR) && (sID != ID_BROADCAST))
//        return false;
//    seedID = sID;
//    return true;
//}
//
//
//// Attempts to set the formation ID to the parameterized formation ID,
//// returning true if successful, false otherwise.
//bool Formation::setFormationID(const int fID)
//{
//    formationID = fID;
//    return true;
//}
//
//
//// Attempts to set the heading to the parameterized heading,
//// returning true if successful, false otherwise.
//bool Formation::setHeading(const float theta)
//{
//    heading = scaleDegrees(theta);
//    return true;
//}
//
//// Returns the function at the parameterized position in this formation.
//Function Formation::getFunction(const int pos) const
//{
//    if ((pos < 0) || (pos > this->size()))
//    	return NULL;
//    return at(pos);
//}
//
//
//// Returns the set of functions of this formation.
//vector<Function> Formation::getFunctions() const
//{
//    return *this;
//}
//
//
//// Returns the radius of this formation.
//float Formation::getRadius() const
//{
//    return radius;
//}
//
//
//// Returns the seed gradient of this formation.
//Vector Formation::getSeedFrp() const
//{
//    return seedFrp;
//}
//
//
//// Returns the seed ID of this formation.
//int Formation::getSeedID() const
//{
//    return seedID;
//}
//
//
//// Returns the formation ID of this formation.
//int Formation::getFormationID() const
//{
//    return formationID;
//}
//
//
//// Returns the heading of this formation.
//float Formation::getHeading() const
//{
//    return heading;
//}
//
//
//// Calculates the intersections of the set of functions of this formation
//// and a circle centered at the parameterized vector position c with
//// the appropriate radius, returning a list of these vectors.
//vector<Vector> Formation::getRelationships(const Vector c)
//{
//    if (empty()) return vector<Vector>();
//    vector<Vector> rels;
//    Function       curr = NULL;
//    for (uint i = 0; i < size(); ++i)
//    {
//        curr = at(i);
//        rels.push_back(calculateDesiredRelationship(curr, -radius, c, heading));
//        rels.push_back(calculateDesiredRelationship(curr,  radius, c, heading));
//    }
//    return rels;
//}
//
//
//// Uses the secant method to calculate the intersection of the function
//// and a circle centered at the parameterized vector position c with
//// the appropriate radius, returning a vector from c to this intersection.
////
//// The secant method is defined by the following recurrence relation:
////
////      x_(n + 1) = x_n - f(x_n) * (x_n - x_(n - 1)) / (f(x_n) - f(x_(n - 1))).
////
//// Returns:     vector from the parameterized vector position c
////              to the intersection of the function and appropriate circle
//// Parameters:
////      f           in      the intersecting function
////      r           in      the radius of the intersecting circle
////      c           in      the position to be centered at
////      theta       in      the rotation of the relationship (default 0)
////
//Vector Formation::calculateDesiredRelationship(const Function f,
//                                  const float  r,
//                                  const Vector   c,
//                                  const float  theta)
//{
//	// TODO: HERE IS THE GOOFY ONE
//    if (f == NULL) return Vector();
//    float   xn        = c.x + r + X_ROOT_THRESHOLD,
//            xn_1      = c.x + r - X_ROOT_THRESHOLD,
//            intersect = 0.0f, error = 0.0f;
//    for (int i = 0; i < X_N_ITERATIONS; ++i)
//    {
//        intersect     = fIntersect(f, r, c, xn);
//        error         = intersect * (xn - xn_1) /
//                       (intersect - fIntersect(f, r, c, xn_1));
//        if (abs(error) <= X_ROOT_THRESHOLD) break;
//        xn_1          = xn;
//        xn           -= error;
//    }
//    Vector rel        = Vector(xn, f(xn)) - c;
//    rel.rotateRelative(-theta);
//    return rel;
//}
//
//
//// Calculates the intersection of the function at the parameterized position
//// and a circle centered at the parameterized vector position c with
//// the appropriate radius, returning a vector from c to this intersection.
//Vector Formation::getRelationship(const int   pos,
//                                  const float r,
//                                  const Vector  c,
//                                  const float theta)
//{
//    return calculateDesiredRelationship(getFunction(pos), r, c, theta);
//}
//
//
//// Copies the contents of the parameterized formation into this formation.
//Formation& Formation::operator =(const Formation &f)
//{
//    setFunctions(f);
//    setRadius(f.radius);
//    setSeedFrp(f.seedFrp);
//    setSeedID(f.seedID);
//    setFormationID(f.formationID);
//    setHeading(f.heading);
//    return *this;
//}
//
//
//// Used to determine the intersection of the parameterized function
//// and a circle centered at the parameterized vector position c and
//// offset by x with the appropriate radius.
//float Formation::fIntersect(const Function f, const float r,
//                              const Vector   c, const float x)
//{
//	float dx = x - c.x;
//	float dy = f(x) - c.y;
//	return  dx * dx + dy * dy - r * r;
//    //return pow(x - c.x, 2.0f) + pow(f(x) - c.y, 2.0f) - pow(r, 2.0f);
//}
