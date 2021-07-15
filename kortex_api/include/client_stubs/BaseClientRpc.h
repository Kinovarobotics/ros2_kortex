#ifndef __BASECLIENT_H__
#define __BASECLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "Base.pb.h"

#include "Frame.h"
#include "IRouterClient.h"
#include "NotificationHandler.h"

#if __cplusplus >= 201402L
#define DEPRECATED [[ deprecated ]]
#define DEPRECATED_MSG(msg) [[ deprecated(msg) ]]
#elif defined(__GNUC__)
#define DEPRECATED __attribute__ ((deprecated))
#define DEPRECATED_MSG(msg) __attribute__ ((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#define DEPRECATED_MSG(msg) __declspec(deprecated(msg))
#else
#define DEPRECATED 
#define DEPRECATED_MSG 
#endif
namespace Kinova
{
namespace Api
{
	namespace Base
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidCreateUserProfile = 0x20001,
			eUidUpdateUserProfile = 0x20002,
			eUidReadUserProfile = 0x20003,
			eUidDeleteUserProfile = 0x20004,
			eUidReadAllUserProfiles = 0x20005,
			eUidReadAllUsers = 0x20006,
			eUidChangePassword = 0x20007,
			eUidCreateSequence = 0x20008,
			eUidUpdateSequence = 0x20009,
			eUidReadSequence = 0x2000a,
			eUidDeleteSequence = 0x2000b,
			eUidReadAllSequences = 0x2000c,
			eUidPlaySequence = 0x2000f,
			eUidPlayAdvancedSequence = 0x20010,
			eUidStopSequence = 0x20011,
			eUidPauseSequence = 0x20012,
			eUidResumeSequence = 0x20013,
			eUidCreateProtectionZone = 0x20014,
			eUidUpdateProtectionZone = 0x20015,
			eUidReadProtectionZone = 0x20016,
			eUidDeleteProtectionZone = 0x20017,
			eUidReadAllProtectionZones = 0x20018,
			eUidCreateMapping = 0x2001a,
			eUidReadMapping = 0x2001b,
			eUidUpdateMapping = 0x2001c,
			eUidDeleteMapping = 0x2001d,
			eUidReadAllMappings = 0x2001e,
			eUidCreateMap = 0x20024,
			eUidReadMap = 0x20025,
			eUidUpdateMap = 0x20026,
			eUidDeleteMap = 0x20027,
			eUidReadAllMaps = 0x20028,
			eUidActivateMap = 0x20029,
			eUidCreateAction = 0x2002a,
			eUidReadAction = 0x2002b,
			eUidReadAllActions = 0x2002c,
			eUidDeleteAction = 0x2002d,
			eUidUpdateAction = 0x2002e,
			eUidExecuteActionFromReference = 0x2002f,
			eUidExecuteAction = 0x20030,
			eUidPauseAction = 0x20031,
			eUidStopAction = 0x20032,
			eUidResumeAction = 0x20033,
			eUidGetIPv4Configuration = 0x2003b,
			eUidSetIPv4Configuration = 0x2003c,
			eUidSetCommunicationInterfaceEnable = 0x2003d,
			eUidIsCommunicationInterfaceEnable = 0x2003e,
			eUidGetAvailableWifi = 0x2003f,
			eUidGetWifiInformation = 0x20040,
			eUidAddWifiConfiguration = 0x20041,
			eUidDeleteWifiConfiguration = 0x20042,
			eUidGetAllConfiguredWifis = 0x20043,
			eUidConnectWifi = 0x20044,
			eUidDisconnectWifi = 0x20045,
			eUidGetConnectedWifiInformation = 0x20046,
			eUidUnsubscribe = 0x20061,
			eUidConfigurationChangeTopic = 0x20062,
			eUidMappingInfoTopic = 0x20063,
			eUidControlModeTopic = 0x20064,
			eUidOperatingModeTopic = 0x20065,
			eUidSequenceInfoTopic = 0x20066,
			eUidProtectionZoneTopic = 0x20067,
			eUidUserTopic = 0x20068,
			eUidControllerTopic = 0x20069,
			eUidActionTopic = 0x2006a,
			eUidRobotEventTopic = 0x2006b,
			eUidPlayCartesianTrajectory = 0x2006d,
			eUidPlayCartesianTrajectoryPosition = 0x2006e,
			eUidPlayCartesianTrajectoryOrientation = 0x2006f,
			eUidStop = 0x20070,
			eUidGetMeasuredCartesianPose = 0x20073,
			eUidSendWrenchCommand = 0x20076,
			eUidSendWrenchJoystickCommand = 0x20077,
			eUidSendTwistJoystickCommand = 0x20078,
			eUidSendTwistCommand = 0x20079,
			eUidPlayJointTrajectory = 0x2007c,
			eUidPlaySelectedJointTrajectory = 0x2007d,
			eUidGetMeasuredJointAngles = 0x2007e,
			eUidSendJointSpeedsCommand = 0x20084,
			eUidSendSelectedJointSpeedCommand = 0x20085,
			eUidSendGripperCommand = 0x20088,
			eUidGetMeasuredGripperMovement = 0x20089,
			eUidSetAdmittance = 0x2008b,
			eUidSetOperatingMode = 0x2008d,
			eUidApplyEmergencyStop = 0x20091,
			eUidClearFaults = 0x20092,
			eUidGetControlMode = 0x20096,
			eUidGetOperatingMode = 0x20097,
			eUidSetServoingMode = 0x20098,
			eUidGetServoingMode = 0x20099,
			eUidServoingModeTopic = 0x2009a,
			eUidRestoreFactorySettings = 0x200a0,
			eUidReboot = 0x200a2,
			eUidFactoryTopic = 0x200a4,
			eUidGetAllConnectedControllers = 0x200a6,
			eUidGetControllerState = 0x200a7,
			eUidGetActuatorCount = 0x200ab,
			eUidStartWifiScan = 0x200ac,
			eUidGetConfiguredWifi = 0x200ad,
			eUidNetworkTopic = 0x200ae,
			eUidGetArmState = 0x200af,
			eUidArmStateTopic = 0x200b0,
			eUidGetIPv4Information = 0x200b1,
			eUidSetWifiCountryCode = 0x200b2,
			eUidGetWifiCountryCode = 0x200b3,
			eUidSetCapSenseConfig = 0x200b4,
			eUidGetCapSenseConfig = 0x200b5,
			eUidGetAllJointsSpeedHardLimitation = 0x200b7,
			eUidGetAllJointsTorqueHardLimitation = 0x200b8,
			eUidGetTwistHardLimitation = 0x200b9,
			eUidGetWrenchHardLimitation = 0x200ba,
			eUidSendJointSpeedsJoystickCommand = 0x200bb,
			eUidSendSelectedJointSpeedJoystickCommand = 0x200bc,
			eUidEnableBridge = 0x200c1,
			eUidDisableBridge = 0x200c2,
			eUidGetBridgeList = 0x200c3,
			eUidGetBridgeConfig = 0x200c4,
			eUidPlayPreComputedJointTrajectory = 0x200c5,
			eUidGetProductConfiguration = 0x200c6,
			eUidUpdateEndEffectorTypeConfiguration = 0x200c9,
			eUidRestoreFactoryProductConfiguration = 0x200ce,
			eUidGetTrajectoryErrorReport = 0x200cf,
			eUidGetAllJointsSpeedSoftLimitation = 0x200d0,
			eUidGetAllJointsTorqueSoftLimitation = 0x200d1,
			eUidGetTwistSoftLimitation = 0x200d2,
			eUidGetWrenchSoftLimitation = 0x200d3,
			eUidSetControllerConfigurationMode = 0x200d4,
			eUidGetControllerConfigurationMode = 0x200d5,
			eUidStartTeaching = 0x200d6,
			eUidStopTeaching = 0x200d7,
			eUidAddSequenceTasks = 0x200d8,
			eUidUpdateSequenceTask = 0x200d9,
			eUidSwapSequenceTasks = 0x200da,
			eUidReadSequenceTask = 0x200db,
			eUidReadAllSequenceTasks = 0x200dc,
			eUidDeleteSequenceTask = 0x2000d,
			eUidDeleteAllSequenceTasks = 0x2000e,
			eUidTakeSnapshot = 0x200df,
			eUidGetFirmwareBundleVersions = 0x200e0,
			eUidExecuteWaypointTrajectory = 0x200e2,
			eUidMoveSequenceTask = 0x200e3,
			eUidDuplicateMapping = 0x200e4,
			eUidDuplicateMap = 0x200e5,
			eUidSetControllerConfiguration = 0x200e6,
			eUidGetControllerConfiguration = 0x200e7,
			eUidGetAllControllerConfigurations = 0x200e8,
			eUidComputeForwardKinematics = 0x200e9,
			eUidComputeInverseKinematics = 0x200ea,
			eUidValidateWaypointList = 0x200eb,
		};
		
		class BaseClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdBase;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			BaseClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			Kinova::Api::Common::UserProfileHandle CreateUserProfile(const FullUserProfile& fulluserprofile, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateUserProfile_callback(const FullUserProfile& fulluserprofile, std::function< void (const Error&, const Kinova::Api::Common::UserProfileHandle&) > callback, uint32_t deviceId = 0);
			std::future<Kinova::Api::Common::UserProfileHandle> CreateUserProfile_async(const FullUserProfile& fulluserprofile, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateUserProfile(const UserProfile& userprofile, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateUserProfile_callback(const UserProfile& userprofile, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateUserProfile_async(const UserProfile& userprofile, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			UserProfile ReadUserProfile(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadUserProfile_callback(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, std::function< void (const Error&, const UserProfile&) > callback, uint32_t deviceId = 0);
			std::future<UserProfile> ReadUserProfile_async(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteUserProfile(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteUserProfile_callback(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteUserProfile_async(const Kinova::Api::Common::UserProfileHandle& userprofilehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			UserProfileList ReadAllUserProfiles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllUserProfiles_callback(std::function< void (const Error&, const UserProfileList&) > callback, uint32_t deviceId = 0);
			std::future<UserProfileList> ReadAllUserProfiles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			UserList ReadAllUsers(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllUsers_callback(std::function< void (const Error&, const UserList&) > callback, uint32_t deviceId = 0);
			std::future<UserList> ReadAllUsers_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ChangePassword(const PasswordChange& passwordchange, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ChangePassword_callback(const PasswordChange& passwordchange, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ChangePassword_async(const PasswordChange& passwordchange, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SequenceHandle CreateSequence(const Sequence& sequence, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateSequence_callback(const Sequence& sequence, std::function< void (const Error&, const SequenceHandle&) > callback, uint32_t deviceId = 0);
			std::future<SequenceHandle> CreateSequence_async(const Sequence& sequence, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateSequence(const Sequence& sequence, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateSequence_callback(const Sequence& sequence, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateSequence_async(const Sequence& sequence, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Sequence ReadSequence(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadSequence_callback(const SequenceHandle& sequencehandle, std::function< void (const Error&, const Sequence&) > callback, uint32_t deviceId = 0);
			std::future<Sequence> ReadSequence_async(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteSequence(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteSequence_callback(const SequenceHandle& sequencehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteSequence_async(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SequenceList ReadAllSequences(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllSequences_callback(std::function< void (const Error&, const SequenceList&) > callback, uint32_t deviceId = 0);
			std::future<SequenceList> ReadAllSequences_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void PlaySequence(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void PlaySequence_callback(const SequenceHandle& sequencehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> PlaySequence_async(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void PlayAdvancedSequence(const AdvancedSequenceHandle& advancedsequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void PlayAdvancedSequence_callback(const AdvancedSequenceHandle& advancedsequencehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> PlayAdvancedSequence_async(const AdvancedSequenceHandle& advancedsequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopSequence(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopSequence_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopSequence_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void PauseSequence(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void PauseSequence_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> PauseSequence_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ResumeSequence(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResumeSequence_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ResumeSequence_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ProtectionZoneHandle CreateProtectionZone(const ProtectionZone& protectionzone, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateProtectionZone_callback(const ProtectionZone& protectionzone, std::function< void (const Error&, const ProtectionZoneHandle&) > callback, uint32_t deviceId = 0);
			std::future<ProtectionZoneHandle> CreateProtectionZone_async(const ProtectionZone& protectionzone, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateProtectionZone(const ProtectionZone& protectionzone, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateProtectionZone_callback(const ProtectionZone& protectionzone, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateProtectionZone_async(const ProtectionZone& protectionzone, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ProtectionZone ReadProtectionZone(const ProtectionZoneHandle& protectionzonehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadProtectionZone_callback(const ProtectionZoneHandle& protectionzonehandle, std::function< void (const Error&, const ProtectionZone&) > callback, uint32_t deviceId = 0);
			std::future<ProtectionZone> ReadProtectionZone_async(const ProtectionZoneHandle& protectionzonehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteProtectionZone(const ProtectionZoneHandle& protectionzonehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteProtectionZone_callback(const ProtectionZoneHandle& protectionzonehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteProtectionZone_async(const ProtectionZoneHandle& protectionzonehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ProtectionZoneList ReadAllProtectionZones(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllProtectionZones_callback(std::function< void (const Error&, const ProtectionZoneList&) > callback, uint32_t deviceId = 0);
			std::future<ProtectionZoneList> ReadAllProtectionZones_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MappingHandle CreateMapping(const Mapping& mapping, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateMapping_callback(const Mapping& mapping, std::function< void (const Error&, const MappingHandle&) > callback, uint32_t deviceId = 0);
			std::future<MappingHandle> CreateMapping_async(const Mapping& mapping, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Mapping ReadMapping(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadMapping_callback(const MappingHandle& mappinghandle, std::function< void (const Error&, const Mapping&) > callback, uint32_t deviceId = 0);
			std::future<Mapping> ReadMapping_async(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateMapping(const Mapping& mapping, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateMapping_callback(const Mapping& mapping, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateMapping_async(const Mapping& mapping, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteMapping(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteMapping_callback(const MappingHandle& mappinghandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteMapping_async(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MappingList ReadAllMappings(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllMappings_callback(std::function< void (const Error&, const MappingList&) > callback, uint32_t deviceId = 0);
			std::future<MappingList> ReadAllMappings_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MapHandle CreateMap(const Map& map, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateMap_callback(const Map& map, std::function< void (const Error&, const MapHandle&) > callback, uint32_t deviceId = 0);
			std::future<MapHandle> CreateMap_async(const Map& map, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Map ReadMap(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadMap_callback(const MapHandle& maphandle, std::function< void (const Error&, const Map&) > callback, uint32_t deviceId = 0);
			std::future<Map> ReadMap_async(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateMap(const Map& map, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateMap_callback(const Map& map, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateMap_async(const Map& map, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteMap(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteMap_callback(const MapHandle& maphandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteMap_async(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MapList ReadAllMaps(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllMaps_callback(const MappingHandle& mappinghandle, std::function< void (const Error&, const MapList&) > callback, uint32_t deviceId = 0);
			std::future<MapList> ReadAllMaps_async(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ActivateMap(const ActivateMapHandle& activatemaphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ActivateMap_callback(const ActivateMapHandle& activatemaphandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ActivateMap_async(const ActivateMapHandle& activatemaphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ActionHandle CreateAction(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateAction_callback(const Action& action, std::function< void (const Error&, const ActionHandle&) > callback, uint32_t deviceId = 0);
			std::future<ActionHandle> CreateAction_async(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Action ReadAction(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAction_callback(const ActionHandle& actionhandle, std::function< void (const Error&, const Action&) > callback, uint32_t deviceId = 0);
			std::future<Action> ReadAction_async(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ActionList ReadAllActions(const RequestedActionType& requestedactiontype, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllActions_callback(const RequestedActionType& requestedactiontype, std::function< void (const Error&, const ActionList&) > callback, uint32_t deviceId = 0);
			std::future<ActionList> ReadAllActions_async(const RequestedActionType& requestedactiontype, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteAction(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteAction_callback(const ActionHandle& actionhandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteAction_async(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateAction(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateAction_callback(const Action& action, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateAction_async(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ExecuteActionFromReference(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ExecuteActionFromReference_callback(const ActionHandle& actionhandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ExecuteActionFromReference_async(const ActionHandle& actionhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ExecuteAction(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ExecuteAction_callback(const Action& action, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ExecuteAction_async(const Action& action, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void PauseAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void PauseAction_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> PauseAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopAction_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ResumeAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResumeAction_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ResumeAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			IPv4Configuration GetIPv4Configuration(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetIPv4Configuration_callback(const NetworkHandle& networkhandle, std::function< void (const Error&, const IPv4Configuration&) > callback, uint32_t deviceId = 0);
			std::future<IPv4Configuration> GetIPv4Configuration_async(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetIPv4Configuration(const FullIPv4Configuration& fullipv4configuration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetIPv4Configuration_callback(const FullIPv4Configuration& fullipv4configuration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetIPv4Configuration_async(const FullIPv4Configuration& fullipv4configuration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCommunicationInterfaceEnable(const CommunicationInterfaceConfiguration& communicationinterfaceconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCommunicationInterfaceEnable_callback(const CommunicationInterfaceConfiguration& communicationinterfaceconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCommunicationInterfaceEnable_async(const CommunicationInterfaceConfiguration& communicationinterfaceconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CommunicationInterfaceConfiguration IsCommunicationInterfaceEnable(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void IsCommunicationInterfaceEnable_callback(const NetworkHandle& networkhandle, std::function< void (const Error&, const CommunicationInterfaceConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<CommunicationInterfaceConfiguration> IsCommunicationInterfaceEnable_async(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WifiInformationList GetAvailableWifi(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAvailableWifi_callback(std::function< void (const Error&, const WifiInformationList&) > callback, uint32_t deviceId = 0);
			std::future<WifiInformationList> GetAvailableWifi_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WifiInformation GetWifiInformation(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetWifiInformation_callback(const Ssid& ssid, std::function< void (const Error&, const WifiInformation&) > callback, uint32_t deviceId = 0);
			std::future<WifiInformation> GetWifiInformation_async(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void AddWifiConfiguration(const WifiConfiguration& wificonfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void AddWifiConfiguration_callback(const WifiConfiguration& wificonfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> AddWifiConfiguration_async(const WifiConfiguration& wificonfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteWifiConfiguration(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteWifiConfiguration_callback(const Ssid& ssid, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteWifiConfiguration_async(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WifiConfigurationList GetAllConfiguredWifis(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllConfiguredWifis_callback(std::function< void (const Error&, const WifiConfigurationList&) > callback, uint32_t deviceId = 0);
			std::future<WifiConfigurationList> GetAllConfiguredWifis_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ConnectWifi(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ConnectWifi_callback(const Ssid& ssid, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ConnectWifi_async(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DisconnectWifi(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DisconnectWifi_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DisconnectWifi_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WifiInformation GetConnectedWifiInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetConnectedWifiInformation_callback(std::function< void (const Error&, const WifiInformation&) > callback, uint32_t deviceId = 0);
			std::future<WifiInformation> GetConnectedWifiInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Unsubscribe(const Kinova::Api::Common::NotificationHandle& notificationhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationConfigurationChangeTopic(std::function< void (ConfigurationChangeNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationMappingInfoTopic(std::function< void (MappingInfoNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function may be removed in a future release. It has been moved to ControlConfig service.") Kinova::Api::Common::NotificationHandle OnNotificationControlModeTopic(std::function< void (ControlModeNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationOperatingModeTopic(std::function< void (OperatingModeNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationSequenceInfoTopic(std::function< void (SequenceInfoNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationProtectionZoneTopic(std::function< void (ProtectionZoneNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationUserTopic(std::function< void (UserNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationControllerTopic(std::function< void (ControllerNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationActionTopic(std::function< void (ActionNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationRobotEventTopic(std::function< void (RobotEventNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void PlayCartesianTrajectory(const ConstrainedPose& constrainedpose, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void PlayCartesianTrajectory_callback(const ConstrainedPose& constrainedpose, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> PlayCartesianTrajectory_async(const ConstrainedPose& constrainedpose, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void PlayCartesianTrajectoryPosition(const ConstrainedPosition& constrainedposition, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void PlayCartesianTrajectoryPosition_callback(const ConstrainedPosition& constrainedposition, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> PlayCartesianTrajectoryPosition_async(const ConstrainedPosition& constrainedposition, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void PlayCartesianTrajectoryOrientation(const ConstrainedOrientation& constrainedorientation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void PlayCartesianTrajectoryOrientation_callback(const ConstrainedOrientation& constrainedorientation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> PlayCartesianTrajectoryOrientation_async(const ConstrainedOrientation& constrainedorientation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Stop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Stop_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Stop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Pose GetMeasuredCartesianPose(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetMeasuredCartesianPose_callback(std::function< void (const Error&, const Pose&) > callback, uint32_t deviceId = 0);
			std::future<Pose> GetMeasuredCartesianPose_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendWrenchCommand(const WrenchCommand& wrenchcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendWrenchCommand_callback(const WrenchCommand& wrenchcommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendWrenchCommand_async(const WrenchCommand& wrenchcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendWrenchJoystickCommand(const WrenchCommand& wrenchcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendWrenchJoystickCommand_callback(const WrenchCommand& wrenchcommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendWrenchJoystickCommand_async(const WrenchCommand& wrenchcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendTwistJoystickCommand(const TwistCommand& twistcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendTwistJoystickCommand_callback(const TwistCommand& twistcommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendTwistJoystickCommand_async(const TwistCommand& twistcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendTwistCommand(const TwistCommand& twistcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendTwistCommand_callback(const TwistCommand& twistcommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendTwistCommand_async(const TwistCommand& twistcommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void PlayJointTrajectory(const ConstrainedJointAngles& constrainedjointangles, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void PlayJointTrajectory_callback(const ConstrainedJointAngles& constrainedjointangles, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> PlayJointTrajectory_async(const ConstrainedJointAngles& constrainedjointangles, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void PlaySelectedJointTrajectory(const ConstrainedJointAngle& constrainedjointangle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void PlaySelectedJointTrajectory_callback(const ConstrainedJointAngle& constrainedjointangle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> PlaySelectedJointTrajectory_async(const ConstrainedJointAngle& constrainedjointangle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			JointAngles GetMeasuredJointAngles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetMeasuredJointAngles_callback(std::function< void (const Error&, const JointAngles&) > callback, uint32_t deviceId = 0);
			std::future<JointAngles> GetMeasuredJointAngles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendJointSpeedsCommand(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendJointSpeedsCommand_callback(const JointSpeeds& jointspeeds, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendJointSpeedsCommand_async(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendSelectedJointSpeedCommand(const JointSpeed& jointspeed, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendSelectedJointSpeedCommand_callback(const JointSpeed& jointspeed, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendSelectedJointSpeedCommand_async(const JointSpeed& jointspeed, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendGripperCommand(const GripperCommand& grippercommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendGripperCommand_callback(const GripperCommand& grippercommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendGripperCommand_async(const GripperCommand& grippercommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Gripper GetMeasuredGripperMovement(const GripperRequest& gripperrequest, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetMeasuredGripperMovement_callback(const GripperRequest& gripperrequest, std::function< void (const Error&, const Gripper&) > callback, uint32_t deviceId = 0);
			std::future<Gripper> GetMeasuredGripperMovement_async(const GripperRequest& gripperrequest, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetAdmittance(const Admittance& admittance, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetAdmittance_callback(const Admittance& admittance, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetAdmittance_async(const Admittance& admittance, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetOperatingMode(const OperatingModeInformation& operatingmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetOperatingMode_callback(const OperatingModeInformation& operatingmodeinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetOperatingMode_async(const OperatingModeInformation& operatingmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ApplyEmergencyStop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ApplyEmergencyStop_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ApplyEmergencyStop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ClearFaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ClearFaults_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ClearFaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function may be removed in a future release. It has been moved to ControlConfig service.") ControlModeInformation GetControlMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("This function may be removed in a future release. It has been moved to ControlConfig service.") void GetControlMode_callback(std::function< void (const Error&, const ControlModeInformation&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("This function may be removed in a future release. It has been moved to ControlConfig service.") std::future<ControlModeInformation> GetControlMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			OperatingModeInformation GetOperatingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetOperatingMode_callback(std::function< void (const Error&, const OperatingModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<OperatingModeInformation> GetOperatingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetServoingMode(const ServoingModeInformation& servoingmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetServoingMode_callback(const ServoingModeInformation& servoingmodeinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetServoingMode_async(const ServoingModeInformation& servoingmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ServoingModeInformation GetServoingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetServoingMode_callback(std::function< void (const Error&, const ServoingModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<ServoingModeInformation> GetServoingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationServoingModeTopic(std::function< void (ServoingModeNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void RestoreFactorySettings(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RestoreFactorySettings_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> RestoreFactorySettings_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Reboot(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Reboot_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Reboot_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationFactoryTopic(std::function< void (FactoryNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControllerList GetAllConnectedControllers(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllConnectedControllers_callback(std::function< void (const Error&, const ControllerList&) > callback, uint32_t deviceId = 0);
			std::future<ControllerList> GetAllConnectedControllers_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControllerState GetControllerState(const ControllerHandle& controllerhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControllerState_callback(const ControllerHandle& controllerhandle, std::function< void (const Error&, const ControllerState&) > callback, uint32_t deviceId = 0);
			std::future<ControllerState> GetControllerState_async(const ControllerHandle& controllerhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ActuatorInformation GetActuatorCount(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetActuatorCount_callback(std::function< void (const Error&, const ActuatorInformation&) > callback, uint32_t deviceId = 0);
			std::future<ActuatorInformation> GetActuatorCount_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StartWifiScan(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StartWifiScan_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StartWifiScan_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WifiConfiguration GetConfiguredWifi(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetConfiguredWifi_callback(const Ssid& ssid, std::function< void (const Error&, const WifiConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<WifiConfiguration> GetConfiguredWifi_async(const Ssid& ssid, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationNetworkTopic(std::function< void (NetworkNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ArmStateInformation GetArmState(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetArmState_callback(std::function< void (const Error&, const ArmStateInformation&) > callback, uint32_t deviceId = 0);
			std::future<ArmStateInformation> GetArmState_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationArmStateTopic(std::function< void (ArmStateNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			IPv4Information GetIPv4Information(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetIPv4Information_callback(const NetworkHandle& networkhandle, std::function< void (const Error&, const IPv4Information&) > callback, uint32_t deviceId = 0);
			std::future<IPv4Information> GetIPv4Information_async(const NetworkHandle& networkhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetWifiCountryCode(const Kinova::Api::Common::CountryCode& countrycode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetWifiCountryCode_callback(const Kinova::Api::Common::CountryCode& countrycode, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetWifiCountryCode_async(const Kinova::Api::Common::CountryCode& countrycode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::CountryCode GetWifiCountryCode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetWifiCountryCode_callback(std::function< void (const Error&, const Kinova::Api::Common::CountryCode&) > callback, uint32_t deviceId = 0);
			std::future<Kinova::Api::Common::CountryCode> GetWifiCountryCode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCapSenseConfig(const CapSenseConfig& capsenseconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCapSenseConfig_callback(const CapSenseConfig& capsenseconfig, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCapSenseConfig_async(const CapSenseConfig& capsenseconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CapSenseConfig GetCapSenseConfig(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCapSenseConfig_callback(std::function< void (const Error&, const CapSenseConfig&) > callback, uint32_t deviceId = 0);
			std::future<CapSenseConfig> GetCapSenseConfig_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") JointsLimitationsList GetAllJointsSpeedHardLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") void GetAllJointsSpeedHardLimitation_callback(std::function< void (const Error&, const JointsLimitationsList&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") std::future<JointsLimitationsList> GetAllJointsSpeedHardLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED JointsLimitationsList GetAllJointsTorqueHardLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void GetAllJointsTorqueHardLimitation_callback(std::function< void (const Error&, const JointsLimitationsList&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<JointsLimitationsList> GetAllJointsTorqueHardLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") TwistLimitation GetTwistHardLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") void GetTwistHardLimitation_callback(std::function< void (const Error&, const TwistLimitation&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicHardLimits from the ControlConfig service instead.") std::future<TwistLimitation> GetTwistHardLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED WrenchLimitation GetWrenchHardLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void GetWrenchHardLimitation_callback(std::function< void (const Error&, const WrenchLimitation&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<WrenchLimitation> GetWrenchHardLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendJointSpeedsJoystickCommand(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendJointSpeedsJoystickCommand_callback(const JointSpeeds& jointspeeds, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendJointSpeedsJoystickCommand_async(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SendSelectedJointSpeedJoystickCommand(const JointSpeed& jointspeed, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SendSelectedJointSpeedJoystickCommand_callback(const JointSpeed& jointspeed, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SendSelectedJointSpeedJoystickCommand_async(const JointSpeed& jointspeed, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			BridgeResult EnableBridge(const BridgeConfig& bridgeconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void EnableBridge_callback(const BridgeConfig& bridgeconfig, std::function< void (const Error&, const BridgeResult&) > callback, uint32_t deviceId = 0);
			std::future<BridgeResult> EnableBridge_async(const BridgeConfig& bridgeconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			BridgeResult DisableBridge(const BridgeIdentifier& bridgeidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DisableBridge_callback(const BridgeIdentifier& bridgeidentifier, std::function< void (const Error&, const BridgeResult&) > callback, uint32_t deviceId = 0);
			std::future<BridgeResult> DisableBridge_async(const BridgeIdentifier& bridgeidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			BridgeList GetBridgeList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetBridgeList_callback(std::function< void (const Error&, const BridgeList&) > callback, uint32_t deviceId = 0);
			std::future<BridgeList> GetBridgeList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			BridgeConfig GetBridgeConfig(const BridgeIdentifier& bridgeidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetBridgeConfig_callback(const BridgeIdentifier& bridgeidentifier, std::function< void (const Error&, const BridgeConfig&) > callback, uint32_t deviceId = 0);
			std::future<BridgeConfig> GetBridgeConfig_async(const BridgeIdentifier& bridgeidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void PlayPreComputedJointTrajectory(const PreComputedJointTrajectory& precomputedjointtrajectory, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void PlayPreComputedJointTrajectory_callback(const PreComputedJointTrajectory& precomputedjointtrajectory, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> PlayPreComputedJointTrajectory_async(const PreComputedJointTrajectory& precomputedjointtrajectory, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::ProductConfiguration::CompleteProductConfiguration GetProductConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetProductConfiguration_callback(std::function< void (const Error&, const Kinova::Api::ProductConfiguration::CompleteProductConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<Kinova::Api::ProductConfiguration::CompleteProductConfiguration> GetProductConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateEndEffectorTypeConfiguration(const Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType& productconfigurationendeffectortype, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateEndEffectorTypeConfiguration_callback(const Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType& productconfigurationendeffectortype, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateEndEffectorTypeConfiguration_async(const Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType& productconfigurationendeffectortype, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void RestoreFactoryProductConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RestoreFactoryProductConfiguration_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> RestoreFactoryProductConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TrajectoryErrorReport GetTrajectoryErrorReport(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetTrajectoryErrorReport_callback(std::function< void (const Error&, const TrajectoryErrorReport&) > callback, uint32_t deviceId = 0);
			std::future<TrajectoryErrorReport> GetTrajectoryErrorReport_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") JointsLimitationsList GetAllJointsSpeedSoftLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") void GetAllJointsSpeedSoftLimitation_callback(std::function< void (const Error&, const JointsLimitationsList&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") std::future<JointsLimitationsList> GetAllJointsSpeedSoftLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED JointsLimitationsList GetAllJointsTorqueSoftLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void GetAllJointsTorqueSoftLimitation_callback(std::function< void (const Error&, const JointsLimitationsList&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<JointsLimitationsList> GetAllJointsTorqueSoftLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") TwistLimitation GetTwistSoftLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") void GetTwistSoftLimitation_callback(std::function< void (const Error&, const TwistLimitation&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("This function will be removed in a future release. Use GetKinematicSoftLimits from the ControlConfig service instead.") std::future<TwistLimitation> GetTwistSoftLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED WrenchLimitation GetWrenchSoftLimitation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void GetWrenchSoftLimitation_callback(std::function< void (const Error&, const WrenchLimitation&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<WrenchLimitation> GetWrenchSoftLimitation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetControllerConfigurationMode(const ControllerConfigurationMode& controllerconfigurationmode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetControllerConfigurationMode_callback(const ControllerConfigurationMode& controllerconfigurationmode, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetControllerConfigurationMode_async(const ControllerConfigurationMode& controllerconfigurationmode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControllerConfigurationMode GetControllerConfigurationMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControllerConfigurationMode_callback(std::function< void (const Error&, const ControllerConfigurationMode&) > callback, uint32_t deviceId = 0);
			std::future<ControllerConfigurationMode> GetControllerConfigurationMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StartTeaching(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StartTeaching_callback(const SequenceTaskHandle& sequencetaskhandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StartTeaching_async(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopTeaching(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopTeaching_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopTeaching_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SequenceTasksRange AddSequenceTasks(const SequenceTasksConfiguration& sequencetasksconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void AddSequenceTasks_callback(const SequenceTasksConfiguration& sequencetasksconfiguration, std::function< void (const Error&, const SequenceTasksRange&) > callback, uint32_t deviceId = 0);
			std::future<SequenceTasksRange> AddSequenceTasks_async(const SequenceTasksConfiguration& sequencetasksconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void UpdateSequenceTask(const SequenceTaskConfiguration& sequencetaskconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void UpdateSequenceTask_callback(const SequenceTaskConfiguration& sequencetaskconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> UpdateSequenceTask_async(const SequenceTaskConfiguration& sequencetaskconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SwapSequenceTasks(const SequenceTasksPair& sequencetaskspair, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SwapSequenceTasks_callback(const SequenceTasksPair& sequencetaskspair, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SwapSequenceTasks_async(const SequenceTasksPair& sequencetaskspair, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SequenceTask ReadSequenceTask(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadSequenceTask_callback(const SequenceTaskHandle& sequencetaskhandle, std::function< void (const Error&, const SequenceTask&) > callback, uint32_t deviceId = 0);
			std::future<SequenceTask> ReadSequenceTask_async(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SequenceTasks ReadAllSequenceTasks(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllSequenceTasks_callback(const SequenceHandle& sequencehandle, std::function< void (const Error&, const SequenceTasks&) > callback, uint32_t deviceId = 0);
			std::future<SequenceTasks> ReadAllSequenceTasks_async(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteSequenceTask(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteSequenceTask_callback(const SequenceTaskHandle& sequencetaskhandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteSequenceTask_async(const SequenceTaskHandle& sequencetaskhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DeleteAllSequenceTasks(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DeleteAllSequenceTasks_callback(const SequenceHandle& sequencehandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DeleteAllSequenceTasks_async(const SequenceHandle& sequencehandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TakeSnapshot(const Snapshot& snapshot, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TakeSnapshot_callback(const Snapshot& snapshot, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TakeSnapshot_async(const Snapshot& snapshot, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			FirmwareBundleVersions GetFirmwareBundleVersions(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetFirmwareBundleVersions_callback(std::function< void (const Error&, const FirmwareBundleVersions&) > callback, uint32_t deviceId = 0);
			std::future<FirmwareBundleVersions> GetFirmwareBundleVersions_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ExecuteWaypointTrajectory(const WaypointList& waypointlist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ExecuteWaypointTrajectory_callback(const WaypointList& waypointlist, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ExecuteWaypointTrajectory_async(const WaypointList& waypointlist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void MoveSequenceTask(const SequenceTasksPair& sequencetaskspair, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void MoveSequenceTask_callback(const SequenceTasksPair& sequencetaskspair, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> MoveSequenceTask_async(const SequenceTasksPair& sequencetaskspair, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MappingHandle DuplicateMapping(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DuplicateMapping_callback(const MappingHandle& mappinghandle, std::function< void (const Error&, const MappingHandle&) > callback, uint32_t deviceId = 0);
			std::future<MappingHandle> DuplicateMapping_async(const MappingHandle& mappinghandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MapHandle DuplicateMap(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DuplicateMap_callback(const MapHandle& maphandle, std::function< void (const Error&, const MapHandle&) > callback, uint32_t deviceId = 0);
			std::future<MapHandle> DuplicateMap_async(const MapHandle& maphandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetControllerConfiguration(const ControllerConfiguration& controllerconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetControllerConfiguration_callback(const ControllerConfiguration& controllerconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetControllerConfiguration_async(const ControllerConfiguration& controllerconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControllerConfiguration GetControllerConfiguration(const ControllerHandle& controllerhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControllerConfiguration_callback(const ControllerHandle& controllerhandle, std::function< void (const Error&, const ControllerConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<ControllerConfiguration> GetControllerConfiguration_async(const ControllerHandle& controllerhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControllerConfigurationList GetAllControllerConfigurations(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllControllerConfigurations_callback(std::function< void (const Error&, const ControllerConfigurationList&) > callback, uint32_t deviceId = 0);
			std::future<ControllerConfigurationList> GetAllControllerConfigurations_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Pose ComputeForwardKinematics(const JointAngles& jointangles, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ComputeForwardKinematics_callback(const JointAngles& jointangles, std::function< void (const Error&, const Pose&) > callback, uint32_t deviceId = 0);
			std::future<Pose> ComputeForwardKinematics_async(const JointAngles& jointangles, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			JointAngles ComputeInverseKinematics(const IKData& ikdata, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ComputeInverseKinematics_callback(const IKData& ikdata, std::function< void (const Error&, const JointAngles&) > callback, uint32_t deviceId = 0);
			std::future<JointAngles> ComputeInverseKinematics_async(const IKData& ikdata, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			WaypointValidationReport ValidateWaypointList(const WaypointList& waypointlist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ValidateWaypointList_callback(const WaypointList& waypointlist, std::function< void (const Error&, const WaypointValidationReport&) > callback, uint32_t deviceId = 0);
			std::future<WaypointValidationReport> ValidateWaypointList_async(const WaypointList& waypointlist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif