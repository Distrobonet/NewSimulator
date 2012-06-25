////
//// Filename:        "Formation.h"
////
//// Description:     This class describes a formation.
////
//
//// preprocessor directives
//#ifndef FORMATION_H
//#define FORMATION_H
//#include <vector>
//
//
//
//
//#include <Simulator/Relationship.h>
//using namespace std;
//
//
//
//// mathematical functional type redefinition -  A Function is a function that takes a float and returns a float
//typedef float (*Function)(const float);
//
//
//
//// global constants
//static const Function DEFAULT_FORMATION_FUNCTION = NULL;
//static const float  DEFAULT_FORMATION_RADIUS   = 1.0f;
//static const double X_ROOT_THRESHOLD           = 5E-7;
//static const int    X_N_ITERATIONS             = 100;
//
//
//
//// describes a formation as a vector of functions
//class Formation: public vector<Function>
//{
//
//    public:
//
//        // <constructors>
//        Formation(const Function f     = DEFAULT_FORMATION_FUNCTION,
//                  const float    r     = DEFAULT_FORMATION_RADIUS,
//                  const Vector   sGrad = Vector(),
//                  const int      sID   = ID_BROADCAST,
//                  const int      fID   = -1,
//                  const float    theta = 0.0f);
//        Formation(vector<Function>   f,
//                  const float    r     = DEFAULT_FORMATION_RADIUS,
//                  const Vector   sGrad = Vector(),
//                  const int      sID   = ID_BROADCAST,
//                  const int      fID   = -1,
//                  const float    theta = 0.0f);
//
//        Formation(const Formation &f);
//        virtual ~Formation(){};
//
//        // <public mutator functions>
//        bool setFunction(const Function f = DEFAULT_FORMATION_FUNCTION);
//        bool setFunctions(const vector<Function> &f);
//        bool addFunction(const Function f = DEFAULT_FORMATION_FUNCTION);
//        bool addFunctions(const vector<Function> &f);
//        bool removeFunction(const int pos = 0);
//        bool removeFunctions();
//        bool setRadius(const float r = DEFAULT_FORMATION_RADIUS);
//        bool setSeedFrp(const Vector sGrad = Vector());
//        bool setSeedID(const int sID = ID_BROADCAST);
//        bool setFormationID(const int fID = -1);
//        bool setHeading(const float theta = 0.0f);
//
//
//
//        // <public accessor functions>
//        Function         getFunction(const int pos = 0) const;
//        vector<Function> getFunctions()                   const;
//        float            getRadius()                      const;
//        Vector           getSeedFrp()                const;
//        int              getSeedID()                      const;
//        int              getFormationID()                 const;
//        float            getHeading()                     const;
//
//        // <public utility functions>
//        vector<Vector> getRelationships(const Vector c = Vector());
//        Vector calculateDesiredRelationship(const Function f = DEFAULT_FORMATION_FUNCTION,
//                               const float  r = DEFAULT_FORMATION_RADIUS,
//                               const Vector   c = Vector(),
//                               const float  theta = 0.0f);
//        Vector getRelationship(const int   pos   = 0,
//                               const float r     = DEFAULT_FORMATION_RADIUS,
//                               const Vector  c     = Vector(),
//                               const float theta = 0.0f);
//
//        // <virtual overloaded operators>
//        virtual Formation& operator =(const Formation &f);
//
//        float radius, heading;
//        Vector  seedFrp;
//        int   seedID, formationID;
//    protected:
//
//        // <protected data members>
//
//        // <protected utility functions>
//        float fIntersect(const Function f = DEFAULT_FORMATION_FUNCTION,
//                           const float  r = DEFAULT_FORMATION_RADIUS,
//                           const Vector   c = Vector(),
//                           const float  x = 0.0f);
//};  // Formation
//
//#endif
