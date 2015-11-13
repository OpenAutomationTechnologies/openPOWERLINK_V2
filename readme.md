openPOWERLINK {#mainpage}
=============

## openPOWERLINK - An Open Source POWERLINK protocol stack

Ethernet POWERLINK is a Real-Time Ethernet field bus system. It is
based on the Fast Ethernet Standard IEEE 802.3.

openPOWERLINK is an Open Source Industrial Ethernet stack implementing the
POWERLINK protocol for Managing Node (MN, POWERLINK Master) and
Controlled Node (CN, POWERLINK Slave). It implements all important features
required by modern POWERLINK devices such as Standard, Multiplexed and
PollResponse Chaining mode of operation, dynamic and static PDO mapping, SDO
via ASnd and SDO via UDP, as well as asynchronous communication via a Virtual
Ethernet interface.

Latest stable version: [2.2.2](\ref sect_revision_v2_2_2)
Current version: [2.3.0](\ref sect_revision_v2_3_0)

## License

openPOWERLINK is Open Source software (OSS) and is licensed under the
BSD license. Some target platform specific parts of the stack are licensed
under other licenses such as, without limitation, the GNU General Public
License Version 2. Please refer to the file's header and the file
[\"license.md\"](\ref page_licenses) for the applicable license and the
corresponding terms and conditions.


## Documentation

The documentation of the openPOWERLINK protocol stack can be found in the
subdirectory "doc". It is written in _markdown_ markup format.
The openPOWERLINK software manual can be generated from the markdown
documentation and the in-source documentation with the tool
[Doxygen](http://www.doxygen.org). Doxygen version 1.8 or higher is required.

To generate the software manual:

      > cd doc/software-manual
      > doxygen

The software manual will be created in HTML format under
`doc/software-manual/html`.

The documentation of the latest stack version is also available online on the
openPOWERLINK website: <http://openpowerlink.sourceforge.net/>


## Support

Support on openPOWERLINK is available via the online discussion forums:

* [Discussion](http://sourceforge.net/p/openpowerlink/discussion/)


## Download

openPOWERLINK can be downloaded from its SourceForge project site:

* Sourcecode: [tar.gz](http://downloads.sourceforge.net/project/openpowerlink/openPOWERLINK/V2.3.0/openPOWERLINK_V2.3.0.tar.gz)


## Contributors

(c) SYSTEC electronic GmbH,
    Am Windrad 2,
    D-08468 Heinsdorfergrund,
    <http://www.systec-electronic.com>

(c) Bernecker + Rainer Industrie Elektronik Ges.m.b.H.,
    B&R Strasse 1,
    A-5142 Eggelsberg,
    <http://www.br-automation.com>

(c) Kalycito Infotech Private Limited,
    <http://www.kalycito.com>


## Links and References

- openPOWERLINK project website:
  <http://sourceforge.net/projects/openpowerlink>
- openCONFIGURATOR project website:
  <http://sourceforge.net/projects/openconf>
- Ethernet POWERLINK Standardization Group:
  <http://www.ethernet-powerlink.org>
