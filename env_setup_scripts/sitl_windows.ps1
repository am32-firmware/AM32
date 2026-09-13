# Native Windows entry point: no Python, ARM SDK or shell PATH setup needed.
[CmdletBinding()]
param(
    [ValidateSet('Build', 'Check', 'Setup')]
    [string]$Action = 'Build',
    [string]$CygwinRoot = $env:AM32_CYGWIN_ROOT,
    [string]$Mirror = 'https://mirrors.kernel.org/sourceware/cygwin/'
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$rootFile = Join-Path $repo 'build/sitl-cygwin-root.txt'

try {
    if (!$CygwinRoot -and (Test-Path -LiteralPath $rootFile)) {
        $CygwinRoot = (Get-Content -LiteralPath $rootFile -Raw).Trim()
    }
    if (!$CygwinRoot) {
        # Prefer existing installations, including the C:\cygwin root used in CI.
        $candidates = @('C:\cygwin64', 'C:\cygwin')
        foreach ($key in @('HKCU:\SOFTWARE\Cygwin\setup', 'HKLM:\SOFTWARE\Cygwin\setup')) {
            $installation = Get-ItemProperty -Path $key -ErrorAction SilentlyContinue
            if ($installation) { $candidates += $installation.rootdir }
        }
        $candidates += (Join-Path $env:LOCALAPPDATA 'AM32-Tools\cygwin64')
        foreach ($candidate in $candidates) {
            if ($candidate -and (Test-Path -LiteralPath (Join-Path $candidate 'bin\bash.exe'))) {
                $CygwinRoot = $candidate
                break
            }
        }
    }
    if (!$CygwinRoot) {
        $CygwinRoot = Join-Path $env:LOCALAPPDATA 'AM32-Tools\cygwin64'
    }
    $CygwinRoot = [IO.Path]::GetFullPath($CygwinRoot)
    $bash = Join-Path $CygwinRoot 'bin\bash.exe'
    $cygpath = Join-Path $CygwinRoot 'bin\cygpath.exe'
    # awk is a Cygwin symlink; native Test-Path can only check its gawk target.
    $missing = @('bash', 'cygpath', 'gcc', 'make', 'gawk', 'cp') | Where-Object {
        !(Test-Path -LiteralPath (Join-Path $CygwinRoot "bin\$_.exe"))
    }
    if ($Action -eq 'Setup' -and $missing) {
        $cache = Join-Path $repo 'downloads\cygwin'
        New-Item -ItemType Directory -Force -Path $cache | Out-Null
        $installer = Join-Path $cache 'setup-x86_64.exe'
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Write-Host 'Downloading the official Cygwin installer...'
        Invoke-WebRequest -UseBasicParsing -Uri 'https://cygwin.com/setup-x86_64.exe' -OutFile $installer
        # Setup verifies the signed package index and package checksums itself.
        # Install only the requested packages and dependencies, without upgrading
        # unrelated packages or changing the machine-wide PATH.
        $setupArgs = @('--quiet-mode', '--no-admin', '--no-shortcuts', '--no-write-registry', '--only-site',
            '--site', $Mirror, '--root', $CygwinRoot, '--local-package-dir', $cache,
            '--packages', 'gcc-core,make')
        # Start-Process joins ArgumentList into a Windows command line.
        $quotedArgs = $setupArgs | ForEach-Object { '"' + $_ + '"' }
        $process = Start-Process -FilePath $installer -ArgumentList $quotedArgs -Wait -PassThru
        if ($process.ExitCode -ne 0) {
            throw "Cygwin setup exited with $($process.ExitCode). See $CygwinRoot\var\log\setup.log.full"
        }
    } elseif ($missing) {
        throw "Missing Cygwin tools ($($missing -join ', ')) in $CygwinRoot. Run env_setup_scripts\sitl_setup_windows.cmd first."
    }

    Write-Host "Cygwin: $CygwinRoot"
    # In particular, keep Git for Windows /usr/bin and bundled ARM tools out
    # of the compiler selection. Pass paths as arguments, never shell code.
    $env:PATH = "$CygwinRoot\bin;$env:PATH"
    $script = & $cygpath -u (Join-Path $PSScriptRoot 'sitl_build.sh')
    if ($LASTEXITCODE -ne 0) { throw 'cygpath failed' }
    $buildAction = if ($Action -eq 'Build') { '--build' } else { '--check' }
    & $bash --noprofile --norc -o igncr $script $buildAction
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    if ($Action -eq 'Setup') {
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $rootFile) | Out-Null
        Set-Content -LiteralPath $rootFile -Value $CygwinRoot -Encoding UTF8
        Write-Host 'Open AM32-SITL.code-workspace in VS Code and press Ctrl+Shift+B.'
    }
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
