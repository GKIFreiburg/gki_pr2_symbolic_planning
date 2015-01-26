/*
 * StateAccessException.cpp
 *
 *  Created on: Oct 29, 2014
 *      Author: andreas
 */

#include <tidyup_state_creators/StateAccessException.h>

namespace tidyup_state_creators
{

StateAccessException::StateAccessException(std::string error_msg):error_msg(error_msg)
{
}

StateAccessException::~StateAccessException() throw()
{
}

const char* StateAccessException::what() const throw()
{
    return error_msg.c_str();
}

} /* namespace tidyup_state_creators */
