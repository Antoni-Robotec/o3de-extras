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
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ImGuiBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>

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
        // AZ_COMPONENT(FingerGripper, "{a29eb4fa-0f6f-11ee-be56-0242ac120002}", AZ::Component);
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
        float GetGripperPosition() const override;
        float GetGripperEffort() const override;
        bool IsGripperNotMoving() const override;
        bool HasGripperReachedGoal() const override;


        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        ManipulationJoints m_fingerJoints;
        bool m_grippingInProgress;
        JointPosition m_desiredPosition;
        JointEffort m_maxEffort;

    };
} // namespace ROS2
