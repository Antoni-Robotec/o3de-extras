/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2Lidar2DSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace
    {
        const char* kLaserScanType = "sensor_msgs::msg::LaserScan";
    }

    void ROS2Lidar2DSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        LidarSensorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2Lidar2DSensorComponent, ROS2SensorComponent>()->Version(2)->Field(
                "lidarBase", &ROS2Lidar2DSensorComponent::m_lidarBase);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2Lidar2DSensorComponent>("ROS2 Lidar 2D Sensor", "Lidar 2D sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2Lidar2DSensorComponent::m_lidarBase,
                        "Lidar configuration",
                        "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent()
        : m_lidarBase(LidarTemplateUtils::Get2DModels())
    {
        TopicConfiguration ls;
        AZStd::string type = kLaserScanType;
        ls.m_type = type;
        ls.m_topic = "ls";
        m_sensorConfiguration.m_frequency = 10.f;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, ls));
    }
    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent(
        const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarBase(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2Lidar2DSensorComponent::Visualize()
    {
        m_lidarBase.VisualizeResults();
    }

    void ROS2Lidar2DSensorComponent::Activate()
    {
        m_lidarBase.Init(GetEntityId());

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[kLaserScanType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_laserScanPublisher = ros2Node->create_publisher<sensor_msgs::msg::LaserScan>(fullTopic.data(), publisherConfig.GetQoS());

        ROS2SensorComponent::Activate();
    }

    void ROS2Lidar2DSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_laserScanPublisher.reset();
        m_lidarBase.Deinit();
    }

    void ROS2Lidar2DSensorComponent::FrequencyTick()
    {
        RaycastResult lastScanResults = m_lidarBase.PerformRaycast();

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::LaserScan();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.angle_min = AZ::DegToRad(m_lidarBase.m_lidarConfiguration.m_lidarParameters.m_minHAngle);
        message.angle_max = AZ::DegToRad(m_lidarBase.m_lidarConfiguration.m_lidarParameters.m_maxHAngle);
        message.angle_increment = (message.angle_max - message.angle_min) /
            aznumeric_cast<float>(m_lidarBase.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements);

        message.range_min = m_lidarBase.m_lidarConfiguration.m_lidarParameters.m_minRange;
        message.range_max = m_lidarBase.m_lidarConfiguration.m_lidarParameters.m_maxRange;
        message.scan_time = 1.f / m_sensorConfiguration.m_frequency;
        message.time_increment = 0.0f;

        message.ranges.assign(lastScanResults.m_ranges.begin(), lastScanResults.m_ranges.end());
        m_laserScanPublisher->publish(message);
    }
} // namespace ROS2
