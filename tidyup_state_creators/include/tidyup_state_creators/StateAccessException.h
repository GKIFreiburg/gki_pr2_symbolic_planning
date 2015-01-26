/*
 * StateAccessException.h
 *
 *  Created on: Oct 29, 2014
 *      Author: andreas
 */

#ifndef STATEACCESSEXCEPTION_H_
#define STATEACCESSEXCEPTION_H_

#include <string>
#include <exception>

namespace tidyup_state_creators
{

class StateAccessException: public std::exception
{
public:
    StateAccessException(std::string erorr_msg);
    ~StateAccessException() throw();

    virtual const char* what() const throw();
private:
    std::string error_msg;
};

} /* namespace tidyup_state_creators */

#endif /* STATEACCESSEXCEPTION_H_ */
