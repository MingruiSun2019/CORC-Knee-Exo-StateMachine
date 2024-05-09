/**
 * \file KEDemoMachine.h
 * \author Vincent Crocher
 * /brief The KEDemoMachine class represents an example implementation of an KE state machine.
 * \version 0.3
 * \date 2021-12-05
 *
 * \copyright Copyright (c) 2020 - 2021
 *
 */
#ifndef KE_SM_H
#define KE_SM_H


#include "StateMachine.h"
#include "RobotKE.h"

// State Classes
#include "KEDemoStates.h"


/**
 * @brief Example implementation of a StateMachine for the Knee Exo Robot class. States should implemented KEDemoState
 *
 */
class KEDemoMachine : public StateMachine {

   public:
    KEDemoMachine();
    ~KEDemoMachine();
    void init();

    RobotKE *robot() { return static_cast<RobotKE*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    //std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif /*KE_SM_H*/
