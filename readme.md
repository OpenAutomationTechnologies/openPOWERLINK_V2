openPOWERLINK {#mainpage}
=============

## openPOWERLINK - An open-source POWERLINK protocol stack

Ethernet POWERLINK is a Real-Time Ethernet field bus system. It is
based on the Fast Ethernet Standard IEEE 802.3.

openPOWERLINK is an Open Source Industrial Ethernet stack implementing the
POWERLINK protocol for Managing Node (MN, POWERLINK Master) and
Controlled Node (CN, POWERLINK Slave). It implements all important features
required by modern POWERLINK devices such as Standard, Multiplexed and
PollResponse Chaining mode of operation, dynamic and static PDO mapping, SDO
via ASnd and SDO via UDP, as well as asynchronous communication via a Virtual
Ethernet interface. openPOWERLINK is licensed under the BSD license.

## License

openPOWERLINK is open-source software (OSS) and is licensed under the
BSD license. Some target platform specific parts of the stack are licensed under
other licenses such as, without limitation, the GNU General Public License Version 2.
Please refer to the file's header and the file [\"license.md\"](\ref page_licenses)
for the applicable license and the corresponding terms and conditions.

## Contributors

(c) SYSTEC electronic GmbH, August-Bebel-Str. 29, D-07973 Greiz,
    http://www.systec-electronic.com

(c) Bernecker + Rainer Industrie Elektronik Ges.m.b.H., B&R Strasse 1, A-5142 Eggelsberg
    http://www.br-automation.com
        
(c) Kalycito Infotech Private Limited
    http://www.kalycito.com

## Documentation

* The documentation of the openPOWERLINK protocol stack can be found in the
  subdirectory "doc". It is written in _markdown_ markup format.
* The openPOWERLINK software manual can be generated from the markdown
  documentation and the in source-code documentation with the tool
  [Doxygen](http://www.doxygen.org). Therefore Doxygen version greater or equal
  1.8 is required. The software manual will be created in HTML format under
  `doc/software-manual/html`.
  
  To generate the software manual:
  
      > cd doc/software-manual
      > doxygen

* Further documentation can be found on the sourceforge website:  
  <http://sourceforge.net/projects/openpowerlink/>
  

## Links and References

- http://sourceforge.net/projects/openpowerlink
- http://sourceforge.net/projects/openconf
- http://www.ethernet-powerlink.org
