openPOWERLINK {#page_openpowerlink}
=============

[TOC]

# openPOWERLINK Key Features {#sect_opl_keyfeature}

- Implements Communication profile EPSG 301 1.1.0
- Data link layer and NMT state machine for Controlled and Managing Nodes
- Configuration Manager for configuration of CNs at run-time
- SDO via UDP and POWERLINK ASnd frames
- Dynamic PDO mapping
- User-configurable object dictionary
- Supports the POWERLINK cycle features async-only CN and multiplexed CN
- Supports the POWERLINK extension PollResponse Chaining according to
  EPSG 302-C V 0.0.3 on MN and CN
- Implemented in plain ANSI C
- Modular software structure for simple portability to different target
  platforms
- Supports target platforms with and without operating system
- Event-driven Communication Abstraction Layer
- Provides Generic API for user-application
- Supports the POWERLINK High Availability extension
  [Managing Node Redundancy](\ref page_mnredundancy)
  according to EPSG 302-A V 1.1.0 Section 3

# openPOWERLINK Release V 2.X {#sect_opl_releases2}

The openPOWERLINK release V2.X is an evolution of the openPOWERLINK V1 stack.
Running the kernel and user layers of openPOWERLINK on different processors in
a multi-processor/multi-platform architecture, required fundamental changes in
the Communication Abstraction Layer. Additionally, the existing code base was
greatly improved so that it is easier to understand and maintain. Due to these
reasons a new major release was started.

Whereas the POWERLINK functionalities of the first V2.X version are comparable
to the release V1 (1.8.2), the code base has fundamentally changed.
- The directory structure is changed to better separate code parts
  - User layer stack code
  - Kernel layer stack code
  - Architecture specific code
  - Additional libraries
  - Demo applications
- The build system is changed to be more flexible (Configurations with user/kernel
  layers on different platforms)
- The code is completely refactored to be more maintainable:
  - Introduce new coding guidelines and refactor the code to conform to this
    coding rules.
  - Split "long monster functions" into better readable smaller functions
  - Completely separate modules of users and kernel layers by a completely new
    designed CAL.
- Add unit test framework to be able to easily add new unit tests
- Add in-source documentation and dynamically create source code documentation
  using the tool [Doxygen](http://www.doxygen.org)

For detailed revision information look at the [revision history](\ref page_revision_history).
