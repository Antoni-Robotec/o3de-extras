/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

#include <AzCore/Component/TickBus.h>
#include <ImGuiBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>

namespace ROS2
{
    //! This component implements finger gripper functionality.
    class FingerGripper
        : public AZ::Component
        , public GripperRequestBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        // @TODO: What should I put in?? I copied from VacuumGripper changing a few chars
        AZ_COMPONENT(FingerGripper, "{b29eb4fa-0f6f-11ef-be56-0242ac120092}", AZ::Component);
        FingerGripper() = default;
        ~FingerGripper() = default;

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        // GripperRequestBus::Handler overrides...
        AZ::Outcome<void, AZStd::string> GripperCommand(float position, float maxEffort) override;
        AZ::Outcome<void, AZStd::string> CancelGripperCommand() override;
        // Sum of all joint positions
        float GetGripperPosition() const override;
        // Sum of all efforts exerted by fingers
        float GetGripperEffort() const override;
        // Non-articulation fingers return 0 effort!
        bool IsGripperNotMoving() const override;
        // Doesn't check if the max force is applied, only checks speed
        bool HasGripperReachedGoal() const override;

        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        ManipulationJoints& GetFingerJoints();
        float GetDefaultPosition();
        void SetPosition(float position, float maxEffort);
        void PublishFeedback() const;

        ManipulationJoints m_fingerJoints;
        bool m_grippingInProgress;
        bool m_initialised;
        float m_desiredPosition;
        float m_maxEffort;
        float m_defaultPosition;

        // @TODO: Should this be an input variable? Maybe just a constant?
        float m_epsilon = 1e-5;
        float m_ImGuiPosition;
    };
} // namespace ROS2
