@echo off
setlocal enabledelayedexpansion

:: ANSI ��ɫ����
set "RED=[91m"
set "GREEN=[92m"
set "YELLOW=[93m"
set "BLUE=[94m"
set "RESET=[0m"

set foundPath=
set binPath=
set exeName=STM32_Programmer_CLI.exe

:: Ԥ�������̷��Ϳ��ܵ�·��
set drives=C D E F
set folderIDE=\ST\STM32CubeIDE*
set folderCLT=\ST\STM32CUbeCLT*
set folderPath=\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_*

:: ���������̷������� STM32_Programmer_CLI.exe
for %%d in (%drives%) do (
    for /d %%p in ("%%d:%folderCLT%") do (
        if exist "%%p\STM32CubeProgrammer\bin\%exeName%" (
            set foundPath=%%p\STM32CubeProgrammer\bin\%exeName%
            goto foundCLI
        )
    )
    for /d %%p in ("%%d:%folderIDE%") do (
        if exist "%%p%folderPath%" (
            for /d %%q in ("%%p%folderPath%") do (
                if exist "%%q\tools\bin\%exeName%" (
                    set foundPath=%%q\tools\bin\%exeName%
                    goto foundCLI
                )
            )
        )
    )
)

:: δ�ҵ� CLI ���ߣ���ʼ�ݹ�����
set searchFolderIDE=com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32*
set searchFolderCLT=STM32CubeProgrammer

echo %YELLOW%�������δ��װ��Ĭ��λ�ã���ʼ�ݹ����%RESET%
echo ���ڴ� D �̲��ң����Ե�Ƭ��......
:: D ��
:: �ݹ����� STM32CubeIDE ����
for /d /r D:\ %%i in (%searchFolderIDE%) do (
    if exist "%%i\tools\bin\%exeName%" (
        echo %GREEN%�ҵ� CLT ���ߣ����Ͽ�ʼ����%RESET%
        set foundPath=%%i\tools\bin\%exeName%
        goto foundCLI
    )
)
:: �ݹ����� CLT ����
for /d /r D:\ %%i in (%searchFolderCLT%) do (
    if exist "%%i\bin\%exeName%" (
        echo %GREEN%�ҵ� CLT ���ߣ����Ͽ�ʼ����%RESET%
        set foundPath=%%i\bin\%exeName%
        goto foundCLI
    )
)

echo �� D ��δ�ҵ�
echo.
echo ���ڴ� C �̲��ң����Ե�Ƭ��......
:: D ��
:: �ݹ����� STM32CubeIDE ����
for /d /r C:\ %%i in (%searchFolderIDE%) do (
    if exist "%%i\tools\bin\%exeName%" (
        echo %GREEN%�ҵ� CLT ���ߣ����Ͽ�ʼ����%RESET%
        set foundPath=%%i\tools\bin\%exeName%
        goto foundCLI
    )
)
:: �ݹ����� CLT ����
for /d /r C:\ %%i in (%searchFolderCLT%) do (
    if exist "%%i\bin\%exeName%" (
        echo %GREEN%�ҵ� CLT ���ߣ����Ͽ�ʼ����%RESET%
        set foundPath=%%i\bin\%exeName%
        goto foundCLI
    )
)

:: ��Ȼδ�ҵ� CLI ���ߣ������˳�
echo %RED%[ERROR]%RESET% STM32_Programmer_CLI.exe not found. Please install STM32CubeProgrammer.
echo %RED%[ERROR]%RESET% ����ʧ�ܣ����ĵ���δ��װ STM32CubeIDE ����������Ȱ�װ�ſ������ء�
echo %RED%[ERROR]%RESET% ����ʧ�ܣ����ĵ���δ��װ STM32CubeIDE ����������Ȱ�װ�ſ������ء�
echo %RED%[ERROR]%RESET% ����ʧ�ܣ����ĵ���δ��װ STM32CubeIDE ����������Ȱ�װ�ſ������ء�
timeout /t 29
exit /b 1

:foundCLI
echo %GREEN%Found STM32_Programmer_CLI:%RESET% %foundPath%

:: �Զ����� bin �ļ�
if exist ".\build\Debug\*.bin" (
    for %%f in (.\build\Debug\*.bin) do (
        set binPath=%%~ff
        goto foundBin
    )
)

if exist ".\Debug\*.elf" (
    for %%f in (.\Debug\*.elf) do (
        set binPath=%%~ff
        goto foundBin
    )
)

:: û�ҵ� bin �ļ��������˳�
echo %RED%[ERROR]%RESET% No bin file found in expected locations!
echo.
echo %RED%[ERROR]%RESET% ����ʧ�ܣ�δ�ҵ������ص��ļ������飺
echo %RED%%RESET% 1���벻Ҫ��ѹ������򿪣������Ƚ�ѹ��������
echo %RED%%RESET% 2�������ļ������������������ر�����
echo %RED%%RESET% 3���˽ű���Ҫ���ڹ��̸�Ŀ¼������
timeout /t 60
exit /b 1

:foundBin
echo %GREEN%Found BIN file:%RESET% %binPath%

:: ���� bin �ļ��� Flash
echo %YELLOW%Flashing STM32 firmware...%RESET%
echo %YELLOW%��ʼ����...%RESET%
"%foundPath%" -c port=SWD -w "%binPath%" 0x08000000
if %errorlevel% neq 0 (
    echo %RED%[ERROR]%RESET% ST���߱�������ʧ��
    timeout /t 60
    exit /b 1
)

:: ���͸�λ����
echo %YELLOW%Flash programming successful. Resetting STM32...%RESET%
"%foundPath%" -c port=SWD -rst
if %errorlevel% neq 0 (
    echo %RED%[ERROR]%RESET% δ�ܸ�λ��Ƭ�������ֶ����¸�λ���������ϵ����
    timeout /t 60
    exit /b 1
)

:: �����ɹ���Ϣ
echo %GREEN%STM32 successfully programmed and reset! [0m
:: ����
echo.
echo %GREEN%���سɹ����� [0m
timeout /t 15
exit


