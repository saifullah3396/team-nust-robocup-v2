/**
 * @file Utils/include/DataHolders/DataHeader.h
 *
 * This file declares and implements the struct DataHeader
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 Jan 2018
 */

#pragma once

/**
 * @struct DataHeader
 * @brief An interface for giving capability to the derived class
 *   to assign itself an id and timestamps
 */
struct DataHeader
{
public:
  /**
   * @brief resetId Assigns a new id to the struct different
   *   from the previous one
   */
  void resetId()
  {
    unsigned prevId = id;
    do { id = rand() % 10 + 1; } while (id == prevId);
  }

  void setTimeStamp(const double& timeStamp) { this->timeStamp = timeStamp; }
  const double getTimeStamp() const { return this->timeStamp; }

  unsigned id = {0}; //! Identity of current observation
  double timeStamp = {0.0}; //! Identity of current observation
};
