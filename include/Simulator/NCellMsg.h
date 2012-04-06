//
// Filename:        "NCellMsg.h"
//
// Programmer:      Brent Beer
// Last modified:   19May2010
//
// Description:     This structure describes a property message that
//                  is used as a state variable to keep track if a property
//                  message that was requested has had a response.
//

#ifndef NCellMSG_H
#define NCellMSG_H
#include <vector>
#include <Simulator/Vector.h>
#include <Simulator/GLIncludes.h>

using namespace std;

struct NCellMsg
{

  // <data members>
  int toID;    //ID of the robot the message was sent to.
  int count;  //count of the number of responses summed up so far.
  //vector gradient;
  bool response; //boolean of wheather or not the msg has been received.

  // <constructor>
  
  //
  // PropMsg(to, from, answer)
  // Last modified: 25Feb2010
  //
  // This is the default constructor that initializes
  // this class to its parameterized values
  //
  // Returns: <none>
  //
  // Parameters: 
  //     toID      in         the default ID of recipient 
  //     count     in         the default count of responses.
  //     response  in         the default of state of a response
  NCellMsg(const int    to        = -1,
          const int     num       = 0,
          const bool      answer    = false)
        : toID(to), count(num), response(answer)
  {
  }//PropMsg(const..{int, int,bool})
}; //PropMsg
#endif
