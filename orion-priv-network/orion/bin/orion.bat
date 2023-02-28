@rem
@rem Copyright 2015 the original author or authors.
@rem
@rem Licensed under the Apache License, Version 2.0 (the "License");
@rem you may not use this file except in compliance with the License.
@rem You may obtain a copy of the License at
@rem
@rem      http://www.apache.org/licenses/LICENSE-2.0
@rem
@rem Unless required by applicable law or agreed to in writing, software
@rem distributed under the License is distributed on an "AS IS" BASIS,
@rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@rem See the License for the specific language governing permissions and
@rem limitations under the License.
@rem

@if "%DEBUG%" == "" @echo off
@rem ##########################################################################
@rem
@rem  orion startup script for Windows
@rem
@rem ##########################################################################

@rem Set local scope for the variables with windows NT shell
if "%OS%"=="Windows_NT" setlocal

set DIRNAME=%~dp0
if "%DIRNAME%" == "" set DIRNAME=.
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%..

@rem Add default JVM options here. You can also use JAVA_OPTS and ORION_OPTS to pass JVM options to this script.
set DEFAULT_JVM_OPTS="-Dvertx.disableFileCPResolving=true" "-Dvertx.logger-delegate-factory-class-name=io.vertx.core.logging.Log4j2LogDelegateFactory"

@rem Find java.exe
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if "%ERRORLEVEL%" == "0" goto init

echo.
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH.
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:findJavaFromJavaHome
set JAVA_HOME=%JAVA_HOME:"=%
set JAVA_EXE=%JAVA_HOME%/bin/java.exe

if exist "%JAVA_EXE%" goto init

echo.
echo ERROR: JAVA_HOME is set to an invalid directory: %JAVA_HOME%
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:init
@rem Get command-line arguments, handling Windows variants

if not "%OS%" == "Windows_NT" goto win9xME_args

:win9xME_args
@rem Slurp the command line arguments.
set CMD_LINE_ARGS=
set _SKIP=2

:win9xME_args_slurp
if "x%~1" == "x" goto execute

set CMD_LINE_ARGS=%*

:execute
@rem Setup the command line

set CLASSPATH=%APP_HOME%\lib\orion-develop.jar;%APP_HOME%\lib\bytes-1.3.0.jar;%APP_HOME%\lib\concurrent-1.3.0.jar;%APP_HOME%\lib\concurrent-coroutines-1.3.0.jar;%APP_HOME%\lib\config-1.3.0.jar;%APP_HOME%\lib\crypto-1.3.0.jar;%APP_HOME%\lib\io-1.3.0.jar;%APP_HOME%\lib\kv-1.3.0.jar;%APP_HOME%\lib\net-1.3.0.jar;%APP_HOME%\lib\rlp-1.3.0.jar;%APP_HOME%\lib\toml-1.3.0.jar;%APP_HOME%\lib\vertx-web-3.7.1.jar;%APP_HOME%\lib\vertx-web-common-3.7.1.jar;%APP_HOME%\lib\vertx-auth-common-3.7.1.jar;%APP_HOME%\lib\vertx-core-3.7.1.jar;%APP_HOME%\lib\bcpkix-jdk15on-1.64.jar;%APP_HOME%\lib\bcprov-jdk15on-1.64.jar;%APP_HOME%\lib\jnr-ffi-2.1.8.jar;%APP_HOME%\lib\leveldbjni-all-1.8.jar;%APP_HOME%\lib\mapdb-3.0.7.jar;%APP_HOME%\lib\lz4-java-1.6.0.jar;%APP_HOME%\lib\bonecp-0.8.0.RELEASE.jar;%APP_HOME%\lib\openjpa-3.1.0.jar;%APP_HOME%\lib\postgresql-42.2.6.jar;%APP_HOME%\lib\toml4j-0.7.2.jar;%APP_HOME%\lib\jackson-datatype-jdk8-2.10.0.jar;%APP_HOME%\lib\jackson-databind-2.10.0.jar;%APP_HOME%\lib\jackson-dataformat-cbor-2.10.0.jar;%APP_HOME%\lib\log4j-slf4j-impl-2.11.2.jar;%APP_HOME%\lib\log4j-core-2.11.2.jar;%APP_HOME%\lib\log4j-api-2.11.2.jar;%APP_HOME%\lib\jaxb-api-2.3.0.jar;%APP_HOME%\lib\ojdbc10-19.3.0.0.jar;%APP_HOME%\lib\kotlinx-coroutines-core-1.4.2.jar;%APP_HOME%\lib\antlr4-runtime-4.7.1.jar;%APP_HOME%\lib\netty-handler-proxy-4.1.34.Final.jar;%APP_HOME%\lib\netty-codec-http2-4.1.34.Final.jar;%APP_HOME%\lib\netty-codec-http-4.1.34.Final.jar;%APP_HOME%\lib\netty-handler-4.1.34.Final.jar;%APP_HOME%\lib\netty-resolver-dns-4.1.34.Final.jar;%APP_HOME%\lib\netty-codec-socks-4.1.34.Final.jar;%APP_HOME%\lib\netty-codec-dns-4.1.34.Final.jar;%APP_HOME%\lib\netty-codec-4.1.34.Final.jar;%APP_HOME%\lib\netty-transport-4.1.34.Final.jar;%APP_HOME%\lib\netty-buffer-4.1.34.Final.jar;%APP_HOME%\lib\netty-resolver-4.1.34.Final.jar;%APP_HOME%\lib\netty-common-4.1.34.Final.jar;%APP_HOME%\lib\jackson-core-2.10.0.jar;%APP_HOME%\lib\vertx-bridge-common-3.7.1.jar;%APP_HOME%\lib\jffi-1.2.17.jar;%APP_HOME%\lib\jffi-1.2.17-native.jar;%APP_HOME%\lib\asm-commons-5.0.3.jar;%APP_HOME%\lib\asm-analysis-5.0.3.jar;%APP_HOME%\lib\asm-util-5.0.3.jar;%APP_HOME%\lib\asm-tree-5.0.3.jar;%APP_HOME%\lib\asm-5.0.3.jar;%APP_HOME%\lib\jnr-x86asm-1.0.2.jar;%APP_HOME%\lib\kotlin-stdlib-1.4.0.jar;%APP_HOME%\lib\eclipse-collections-forkjoin-11.0.0.jar;%APP_HOME%\lib\eclipse-collections-11.0.0.jar;%APP_HOME%\lib\eclipse-collections-api-11.0.0.jar;%APP_HOME%\lib\guava-27.0.1-jre.jar;%APP_HOME%\lib\elsa-3.0.0-M5.jar;%APP_HOME%\lib\slf4j-api-1.7.25.jar;%APP_HOME%\lib\commons-collections4-4.3.jar;%APP_HOME%\lib\serp-1.15.1.jar;%APP_HOME%\lib\geronimo-jms_1.1_spec-1.1.1.jar;%APP_HOME%\lib\geronimo-jta_1.1_spec-1.1.1.jar;%APP_HOME%\lib\commons-pool2-2.6.0.jar;%APP_HOME%\lib\xbean-asm7-shaded-4.12.jar;%APP_HOME%\lib\geronimo-jpa_2.2_spec-1.0.jar;%APP_HOME%\lib\gson-2.8.1.jar;%APP_HOME%\lib\jackson-annotations-2.10.0.jar;%APP_HOME%\lib\ucp-19.3.0.0.jar;%APP_HOME%\lib\oraclepki-19.3.0.0.jar;%APP_HOME%\lib\osdt_cert-19.3.0.0.jar;%APP_HOME%\lib\osdt_core-19.3.0.0.jar;%APP_HOME%\lib\simplefan-19.3.0.0.jar;%APP_HOME%\lib\ons-19.3.0.0.jar;%APP_HOME%\lib\kotlin-stdlib-common-1.4.0.jar;%APP_HOME%\lib\failureaccess-1.0.1.jar;%APP_HOME%\lib\listenablefuture-9999.0-empty-to-avoid-conflict-with-guava.jar;%APP_HOME%\lib\jsr305-3.0.2.jar;%APP_HOME%\lib\checker-qual-2.5.2.jar;%APP_HOME%\lib\error_prone_annotations-2.2.0.jar;%APP_HOME%\lib\j2objc-annotations-1.1.jar;%APP_HOME%\lib\animal-sniffer-annotations-1.17.jar;%APP_HOME%\lib\annotations-13.0.jar

@rem Execute orion
"%JAVA_EXE%" %DEFAULT_JVM_OPTS% %JAVA_OPTS% %ORION_OPTS%  -classpath "%CLASSPATH%" net.consensys.orion.cmd.Orion %CMD_LINE_ARGS%

:end
@rem End local scope for the variables with windows NT shell
if "%ERRORLEVEL%"=="0" goto mainEnd

:fail
rem Set variable ORION_EXIT_CONSOLE if you need the _script_ return code instead of
rem the _cmd.exe /c_ return code!
if  not "" == "%ORION_EXIT_CONSOLE%" exit 1
exit /b 1

:mainEnd
if "%OS%"=="Windows_NT" endlocal

:omega
