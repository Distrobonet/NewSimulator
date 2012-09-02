//
// Filename:        "Relationship.h"
//
// Description:     This structure defines a desired and actual vector
//                  relationship to a robot with the specified ID number.
//

// preprocessor directives
#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H
#include <vector>
#include <Simulator/PhysicsVector.h>

using namespace std;



// global constants
static const int ID_NO_NBR = -1;



// defines a desired and actual vector relationship
// to a robot with the specified ID number
struct Relationship
{

//    // <data members>
//    Vector relDesired, relActual;
//    int  ID;
//
//
//
//    // <constructors>
//
//
//    // Default constructor that initializes
//    // this relationship to the parameterized values.
//    Relationship(const Vector desired  = Vector(),
//                 const Vector actual   = Vector(),
//                 const int  id       = ID_NO_NBR)
//        : relDesired(desired), relActual(actual), ID(id)
//    {
//    }   // Relationship(const Vector, const Vector, const int)
//
//
//
//    // <utility functions>
//
//    // Returns the difference (error) between desired and actual relationships.
//    Vector getError()
//    {
//        return relDesired - relActual;
//    }   // getError()
//
//
//
//    // <friend functions>
//
//
//    // Returns a reference to the relationship with the parameterized ID
//    // within the parameterized list, NULL otherwise.
//    friend Relationship* relWithID(vector<Relationship> &rels,
//                                   const int           id)
//    {
//        for (int i = 0; i < (int)rels.size(); ++i)
//            if (rels[i].ID == id) return &rels[i];
//        return 0;
//    }   // relWithID(const LinkedList<Relationship> &, const int)
};  // Relationship

#endif
