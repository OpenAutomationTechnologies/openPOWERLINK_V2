/**
********************************************************************************
\file   notify.h

\brief  Declaration of the CNotify

This file contains the class declaration for the CNotify class which implements
the INetCfgComponent entry points for the NDIS intermediate driver's notify
object.

\ingroup module_notify_ndisim
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

#ifndef _INC_notify_H_
#define _INC_notify_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <netcfgn.h>
#include <list>

#include <notifyObj_i.h>

#include "resource.h"       // main symbols
#include "common.h"
#include "phyAdapter.h"


#if defined(_WIN32_WCE) && !defined(_CE_DCOM) && !defined(_CE_ALLOW_SINGLE_THREADED_OBJECTS_IN_MTA)
#error "Single-threaded COM objects are not properly supported on Windows CE platform, such as the Windows Mobile platforms that do not include full DCOM support. Define _CE_ALLOW_SINGLE_THREADED_OBJECTS_IN_MTA to force ATL to support creating single-thread COM object's and allow use of it's single-threaded COM object implementations. The threading model in your rgs file was set to 'Free' as that is the only threading model supported in non DCOM Windows CE platforms."
#endif

using namespace ATL;

//------------------------------------------------------------------------------
/**
\class CNotify

\brief Notify class implements the INetCfgComponent routines

This is the top level class for the notify object and implements the
INetCfgComponent abstract methods.
*/
//------------------------------------------------------------------------------
class ATL_NO_VTABLE CNotify :
    public CComObjectRootEx<CComSingleThreadModel>,
    public CComCoClass<CNotify, &CLSID_CNotify>,
    public ISupportErrorInfo,
    public IDispatchImpl<INotify, &IID_INotify, &LIBID_notifyObjLib, /*wMajor =*/ 1, /*wMinor =*/ 0>,
    public INetCfgComponentControl,
    public INetCfgComponentSetup,
    public INetCfgComponentNotifyBinding,
    public INetCfgComponentNotifyGlobal
{
public:
    //-------------------------------------------------------------------------
    /**
    \brief Constructor for class CNotify

    */
    //-------------------------------------------------------------------------
    CNotify();

    //-------------------------------------------------------------------------
    /**
    \brief Destructor for class CNotify

    */
    //-------------------------------------------------------------------------
    ~CNotify();

    BEGIN_COM_MAP(CNotify)
        COM_INTERFACE_ENTRY(INotify)
        COM_INTERFACE_ENTRY(IDispatch)
        COM_INTERFACE_ENTRY(ISupportErrorInfo)
        COM_INTERFACE_ENTRY(INetCfgComponentControl)
        COM_INTERFACE_ENTRY(INetCfgComponentSetup)
        COM_INTERFACE_ENTRY(INetCfgComponentNotifyBinding)
        COM_INTERFACE_ENTRY(INetCfgComponentNotifyGlobal)
    END_COM_MAP()

    DECLARE_REGISTRY_RESOURCEID(IDR_NOTIFY)

    //-------------------------------------------------------------------------
    /**
    \brief Indicates whether an interface supports the IErrorInfo interface.

    \param  riid_p      An interface identifier (IID).

    \return HRESULT error code.
    \retval S_OK, The interface supports IErrorInfo.
    \retval S_FALSE, The interface does not support IErrorInfo.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(InterfaceSupportsErrorInfo)(REFIID riid_p);

    //-------------------------------------------------------------------------
    // INetCfgComponentControl
    //
    // The following functions provide the INetCfgComponentControl interface.
    //
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    /**
    \brief  Initialize the notify object

    The Initialize method initializes the notify object of the network
    component and provides access to the component and to all aspects
    of network configuration.

    \param  pIComp_p        Pointer to INetCfgComponent object.
    \param  pINetCfg_p      Pointer to INetCfg object.
    \param  fInstalling_p   TRUE if the miniport is being installed.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(Initialize)(IN INetCfgComponent* pIComp_p, IN INetCfg* pINetCfg_p,
                          IN BOOL fInstalling_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Cancel any changes made to internal data

    The CancelChanges method directs the notify object of the network
    component to disregard any proposed changes for the component's network
    configuration state.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(CancelChanges)();

    //-------------------------------------------------------------------------
    /**
    \brief  Apply changes for registry

    The ApplyRegistryChanges method directs the notify object of the network
    component to apply to the operating system the proposed changes for the
    component's network configuration state.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval S_FALSE, if no changes were made.
    \retval NETCFG_S_REBOOT, if the applied changes require a system reboot.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(ApplyRegistryChanges)();

    //-------------------------------------------------------------------------
    /**
    \brief  Apply changes in the PNP state

    The ApplyPnpChanges method informs the notify object of the network
    component that it can send configuration information to the component's
    driver.

    \param  pICallback_p    Pointer to the INetCfgPnpReconfigCallback interface
                            for configuring the driver of the network component
                            that owns the notify object.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(ApplyPnpChanges)(IN INetCfgPnpReconfigCallback* pICallback_p);

    //----------------------------------------------------------------------------
    // INetCfgComponentSetup
    //
    // The following functions provide the INetCfgComponentSetup interface.
    //
    //----------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    /**
    \brief  Installation method for the network component

    The Install method directs the notify object of the network component
    to perform operations required to install the component.

    \param  setupFlags_p    Setup flags indicating type of installation.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_S_REBOOT, A system reboot is required after Install
                             performs installation operations.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(Install)(IN DWORD setupFlags_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Upgrade method for network component

    The Upgrade method directs the notify object of the network component
    to perform the operations required when the operating system changes
    to a new, improved, or different version.

    \param  setupFlags_p            Specifies the type of operating system
                                    from which to upgrade.
    \param  upgradeFromBuildNo_p    Build number from which to upgrade.

    \note Not implemented in the current version.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(Upgrade)(IN DWORD setupFlags_p, IN DWORD upgradeFromBuildNo_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Read settings from answer file and configure CNotify

    The ReadAnswerFile method directs the notify object of the network component
    to open a specific file for unattended setup and retrieve the parameters
    required to configure the network component.

    \param  pszAnswerFile_p     Name of AnswerFile.
    \param  pszAnswerSection_p  Name of parameters section.

    \note The method is deprecated and is not implemented.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(ReadAnswerFile)(IN PCWSTR pszAnswerFile_p, IN PCWSTR pszAnswerSection_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Cleanup method for the network component

    The Removing method directs the notify object of the network component to
    perform operations required for the component's removal.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(Removing)();

    //----------------------------------------------------------------------------
    // INetCfgComponentNotifyBinding
    //
    // The following functions provide the INetCfgComponentNotifyBinding interface.
    //
    //----------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    /**
    \brief  Accept or reject a new binding path for the network component

    The QueryBindingPath method informs the notify object that a binding path
    is about to be added to its network component. This method directs the
    notify object to evaluate the change and to accept or reject it.

    \param  changeFlag_p        Type of binding change.
    \param  pNetCfgBindPath_p   Pointer to INetCfgBindingPath object.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_S_DISABLE_QUERY, if the notify object accepts the
                                    proposed binding path.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(QueryBindingPath)(IN DWORD changeFlag_p,
                                IN INetCfgBindingPath* pNetCfgBindPath_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Bind to the component passed by OS

    The NotifyBindingPath method informs the notify object that a change
    occurred to the binding path of its network component. This method
    directs the notify object to perform operations related to the change.

    \param  changeFlag_p        Type of binding change.
    \param  pNetCfgBindPath_p   Pointer to INetCfgBindingPath object.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(NotifyBindingPath)(IN DWORD changeFlag_p,
                                 IN INetCfgBindingPath* pNetCfgBindPath_p);

    //----------------------------------------------------------------------------
    // INetCfgComponentNotifyGlobal
    //
    // The following functions provide the INetCfgComponentNotifyGlobal interface.
    //
    //----------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    /**
    \brief  Specify the supported notifications by the network component

    The GetSupportedNotifications method retrieves the types of notifications
    that the notify object of the network component requires from the network
    configuration subsystem.

    \param  pNotificationFlag_p     Pointer to supported notifications.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(GetSupportedNotifications)(OUT DWORD* pNotificationFlag_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Enable or Disable a binding path.

    The SysQueryBindingPath method informs the notify object of the component
    that the addition of a binding path is about to occur. This method also
    directs the object to evaluate the change and to accept or reject it.

    \param  changeFlag_p        Type of binding change.
    \param  pNetCfgBindPath_p   Pointer to INetCfgBindingPath object.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_S_DISABLE_QUERY, if the notify object accepts the
                                    proposed binding path.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(SysQueryBindingPath)(IN DWORD changeFlag_p,
                                   IN INetCfgBindingPath* pNetCfgBindPath_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Informs the notify object of the new bindings changes.

    The SysNotifyBindingPath method informs the notify object of the network
    component that a binding path change occurred. This method directs the
    notify object to perform operations related to the change.

    \param  changeFlag_p        Type of binding change.
    \param  pNetCfgBindPath_p   Pointer to INetCfgBindingPath object.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(SysNotifyBindingPath)(IN DWORD changeFlag_p,
                                    IN INetCfgBindingPath* pNetCfgBindPath_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Informs the notify object of the addition or removal of a component.

    The SysNotifyComponent method informs the notify object of the network
    component that another component has been installed or removed. This
    method directs the notify object to perform operations related to the change.

    \param  changeFlag_p        Type of binding change.
    \param  pNetCfgBindPath_p   Pointer to INetCfgBindingPath object.

    \note  The method is deprecated and not implemented.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    STDMETHOD(SysNotifyComponent)(IN DWORD changeFlag_p,
                                  IN INetCfgComponent* pNetCfgComponent);

    DECLARE_PROTECT_FINAL_CONSTRUCT()

private:
    INetCfgComponent*       pInetComponent;     ///< Protocol's Net Config component
    INetCfg*                pInetCfg;           ///< Protocol's Net Config structure
    tConfigAction           applyAction;        ///< Pointer to hold actions to be applied
    std::list<CPhyAdapter*> pPhyAdapterList;    ///< List of physical adapters

    //-------------------------------------------------------------------------
    /**
    \brief  Initialize adapter list

    Initialize the existing adapter list to perform managing operations.

    \return HRESULT error code.
    \retval S_OK, if successful.
    \retval NETCFG_* codes that are defined in Netcfgx.h, if failed.

    */
    //-------------------------------------------------------------------------
    HRESULT initalizeAdapters(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief Search the upper and lower binding components in the specified path

    The routine enumerates the bind path to identify the lower and upper
    binding components in the specified network binding path. For a intermediate
    driver path, the upper components include protocols attached and the
    lower components include the list of NIC miniport driver to which the
    intermediate driver can bind with.

    \param  pInetBindPath_p     Pointer to the binding path.
    \param  ppUpperComponent_p  Double pointer to receive upper component.
    \param  ppLowerComponent_p  Double pointer to receive lower component.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT getUpperAndLowerBindings(INetCfgBindingPath* pInetBindPath_p,
                                     INetCfgComponent** ppUpperComponent_p,
                                     INetCfgComponent** ppLowerComponent_p);

    //-------------------------------------------------------------------------
    /**
    \brief Add a new adapter to the adapter list

    Create an instance representing the physical adapter and install a virtual
    miniport.

    \param  pAdapter_p  Pointer to the physical adapter to add.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT addAdapter(INetCfgComponent* pAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Remove adapter from the list

    Deletes the instance representing the physical adapter and uninstalls all
    the virtual miniports.

    This function is called when the adapter or the protocol being uninstalled.

    \param  pAdapter_p  Pointer to the physical adapter to remove.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT removeAdapter(INetCfgComponent* pAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Add a new virtual Ethernet miniport

    Installs a virtual Ethernet miniport instance.

    \param  pPhyAdapter_p       Pointer to the physical adapter instance to
                                which miniport will link.
    \param  pAdapterGuid_p      Pointer to adapter GUID.
    \param  pInetComponent_p    Pointer to physical instance of adapter.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT addMiniport(CPhyAdapter* pPhyAdapter_p, GUID* pAdapterGuid_p,
                        INetCfgComponent* pInetComponent_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Remove miniport instance

    Uninstall the virtual Ethernet miniport instance and remove the structures.

    \param  pPhyAdapter_p       Pointer to the physical adapter for which the
                                miniport is to be removed.
    \param  pAdapterGuid_p      Pointer to adapter GUID.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT removeMiniport(CPhyAdapter* pPhyAdapter_p, GUID* pAdapterGuid_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Enable/Disable the bindings of other protocols to the physical adapter.

    \param  pAdapter_p      Pointer to the physical adapter.
    \param  fEnable_p       Flag to identify Enable/Disable.
                            Enable if TRUE, Disable if FALSE.

    */
    //-------------------------------------------------------------------------
    void enableProtocolBindings(INetCfgComponent* pAdapter_p, BOOL fEnable_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Checks if a given binding path contains openPOWERLINK protocol driver

    \param  pBindPath_p     Pointer to binding path.

    \return The routine a BOOL value.
    \retval TRUE, if the openPOWERLINK protocol driver exists.
    \retval FALSE, otherwise.

    */
    //-------------------------------------------------------------------------
    BOOL checkBindingStatus(INetCfgBindingPath* pBindPath_p);

    //-------------------------------------------------------------------------
    /**
    \brief Search the specified adapter instance

    The routine searches the local adapter list to retrieve the adapter instance
    for the specified adapter GUID.

    \param  pGuidAdapter_p  Pointer to the adapter GUID to search.
    \param  ppPhyAdapter_p  Double pointer to receive the adapter instance.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT findAdapter(GUID* pGuidAdapter_p, CPhyAdapter** ppPhyAdapter_p);
};

OBJECT_ENTRY_AUTO(__uuidof(CNotify), CNotify)

#endif // _INC_notify_H_
