README for ti mbedTLS 
======================
*basede on a development version (post 3.2.1)

This folder contains a porting files which hold the function callbacks used by the mbedtls library and the makefiles needed to compile the library.

In order to compile the mbedtls library do the folowing steps:

1) Add ti folder to < mbedtls-development -root-folder> 
    There is a pre-built lib located in the mbedTLS dirctory:
    <mbedtls-root-folder>\ti\lib\ccs\m4\mbedtls.a
    And can be linked without compiling again.<BR>
2)  <b>If you wish to compile yourself - in the Windows command prompt go to: </b><br> 
    <mbedtls-root-folder>\ti<br>
    run:<BR> 
    <i><xdctools-latest-core-install-dir>\gmake.exe SDK\_INSTALL\_DIR=<simplelink-cc32xx-sdk-install-dir>
	</i><br>
	\* This will build the library for CCS, GCC, IAR and TICLANG (under the <toolchain>/m4/ folder)<br>
    e.g. <i>xdctools_3\_62\_01\_15\_core\gmake.exe SDK\_INSTALL\_DIR=simplelink\_cc32xx\_sdk\_6.40.00.00 </i>
	<br>
	<b>To build only for sepcific toolchain, run:</b><br> 
    <i><xdctools-latest_core-install-dir\>\gmake.exe SDK\_INSTALL\_DIR=<simplelink-cc32xx-sdk-install-dir> <tool-chain-folder>/br/mbedtls.a</i><br>
    e.g. <i>xdctools_3\_62\_01\_15\_core\gmake.exe SDK\_INSTALL\_DIR=simplelink\_cc32xx\_sdk\_6.40.00.00 gcc/m4/mbedtls.a</i>
	 

make sure all .c files located in < mbedtls-root-folder>\library are listed in ti\defs.mak as .o
(defs.mak is included in the makefile specefying what fils to compile)

- If you wish to configure your own mbedtls configuration you can change them in:
< mbedtls-development -root-folder>\library\mbedtls_config.h (A pre-configured file to work with tls1.3 is already configured and a copy is attached)