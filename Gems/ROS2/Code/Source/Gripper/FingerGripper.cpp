/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root
* of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "FingerGripper.h"
#include "Utils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

#include "Source/ArticulationLinkComponent.h"
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <imgui/imgui.h>

namespace ROS2 {
    void FingerGripper::Activate()
    {
        // auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        // AZ_Assert(ros2Frame, "Missing Frame Component!");
        // AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_followTrajectoryActionName);
        // m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(namespacedAction, GetEntityId());
        m_grippingInProgress = false;
        AZ::TickBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());

        GetFingerJoints();
    }
    void FingerGripper::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        GripperRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    ManipulationJoints& FingerGripper::GetFingerJoints()
    {
        if (m_fingerJoints.empty())
        {
            JointsManipulationRequestBus::EventResult(m_fingerJoints, GetEntityId(), &JointsManipulationRequests::GetJoints);
        }
        return m_fingerJoints;
    }

    AZ::Outcome<void, AZStd::string> FingerGripper::GripperCommand(float position, float maxEffort) {

        m_grippingInProgress = true;
        m_desiredPosition = position;
        m_maxEffort = maxEffort;

        // Set the position and max force for all fingers
        // Requires adding a max force setter?
    }

    AZ::Outcome<void, AZStd::string> FingerGripper::CancelGripperCommand()
    {
        m_grippingInProgress = false;
        // Set a "default (release)" position on fingers?
        return AZ::Success();
    }

    float GetGripperPosition() const {
        // Sum of all finger positions
        // Sum is what we want for 2 finger grippers, should be okay for grippers with more fingers
        // Should be numerically stable, as they are supposed to all be non-negative

        JointPositions result;
        JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetAllJointPositions);

        float gripperPosition = 0;
        for (const auto& [jointName, position] : result) {
            gripperPosition += position;
        }

        return gripperPosition;
    }

    float GetGripperEffort() const {
        // Sum of all efforts exerted by fingers
        // Non-articulation fingers return 0 effort!

        // Here I need the code from my PR...
    }

    bool IsGripperNotMoving() const {
        // Here I also need the code from my PR...
    }

    bool HasGripperReachedGoal() const {
        return !m_grippingInProgress || GetGripperPosition() == m_desiredPosition;
    }

    void OnTick(float delta, AZ::ScriptTimePoint timePoint) {
        PublishFeedback();
    }
}