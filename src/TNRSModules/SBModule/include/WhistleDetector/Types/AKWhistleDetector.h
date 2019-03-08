/**
 * @file SBModule/include/WhistleDetector/Types/AKWhistleDetector.h
 *
 * This file declares the class AKWhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 21 2018
 */

#pragma once

#include "SBModule/include/WhistleDetector/WhistleDetector.h"

class AlsaRecorder;
struct ProcessingRecord;
class STFT;

/**
 * @class AKWhistleDetector
 * @brief Whistle detector based on Austrian Kangaroos Opensource
 *   whistle detector
 */ 
class AKWhistleDetector : public WhistleDetector
{
public:
  /**
   * @brief AKWhistleDetector Constructor
   * 
   * @param sbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   */
  AKWhistleDetector(SBModule* sbModule, const boost::shared_ptr<SBWDConfig>& config);

  /**
   * @brief ~AKWhistleDetector Destructor
   */
  ~AKWhistleDetector() final;

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;

private:
  int runFrequencyExtraction();
  void stopListening();
  void setupExecution();
  AlsaRecorder *reader;
  STFT* stft;
  static ProcessingRecord config;  
};

typedef boost::shared_ptr<AKWhistleDetector> AKWhistleDetectorPtr;
