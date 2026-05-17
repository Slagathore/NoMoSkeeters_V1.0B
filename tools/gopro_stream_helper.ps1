# tools/gopro_stream_helper.ps1
#
# GoPro Hero 13 stream control helpers for NoMoSkeeters development.
# Dot-source this in your PowerShell session:  . .\tools\gopro_stream_helper.ps1
# Then call the functions directly.

# Default camera IP changes per USB session — set this at the top.
$Global:GoProCameraIP = "172.27.109.51"

function Find-GoPro {
    <#
    .SYNOPSIS
    Scans local network adapters for the GoPro RNDIS interface and probes
    the camera at .51 of the discovered /24. Sets $Global:GoProCameraIP.
    #>
    $candidates = Get-NetIPAddress -AddressFamily IPv4 |
        Where-Object { $_.IPAddress -match '^172\.\d+\.\d+\.\d+$' } |
        Select-Object IPAddress, InterfaceAlias
    foreach ($c in $candidates) {
        $octets = $c.IPAddress.Split('.')
        $candidate = "$($octets[0]).$($octets[1]).$($octets[2]).51"
        Write-Host "Probing $candidate via $($c.InterfaceAlias)..." -ForegroundColor Cyan
        try {
            $r = Invoke-RestMethod -Uri "http://${candidate}:8080/gopro/camera/state" `
                -TimeoutSec 2 -ErrorAction Stop
            if ($r.status) {
                Write-Host "Found GoPro at $candidate" -ForegroundColor Green
                $Global:GoProCameraIP = $candidate
                return $candidate
            }
        } catch {
            # not this one
        }
    }
    Write-Host "No GoPro found on any 172.X.Y.51" -ForegroundColor Red
    return $null
}

function Start-GoProStream {
    [CmdletBinding()]
    param(
        [string]$CameraIP = $Global:GoProCameraIP,
        [ValidateSet("display", "record", "headless")]
        [string]$Mode = "display",
        [string]$RecordPath = "fixtures\gopro_capture_$(Get-Date -Format 'yyyyMMdd_HHmmss').ts"
    )

    # Always stop any stale stream first
    try {
        Invoke-RestMethod "http://${CameraIP}:8080/gopro/camera/stream/stop" `
            -TimeoutSec 5 | Out-Null
    } catch {
        Write-Warning "Couldn't reach camera at $CameraIP — is USB connected?"
        return
    }
    Start-Sleep -Milliseconds 500

    # Kill any consumer on 8554
    Get-Process ffplay, ffmpeg -ErrorAction SilentlyContinue | Stop-Process -Force

    # Launch consumer BEFORE asking camera to push.
    # CRITICAL: use udp://0.0.0.0:8554 not udp://@:8554 — newer ffmpeg
    # builds resolve @ to IPv6 [::] on Windows, which silently breaks.
    $ffArgs = "-fflags nobuffer -flags low_delay -framedrop -i udp://0.0.0.0:8554"

    switch ($Mode) {
        "display"  {
            Start-Process ffplay -ArgumentList $ffArgs
        }
        "record"   {
            Start-Process ffmpeg -ArgumentList "$ffArgs -c copy -f mpegts $RecordPath"
            Write-Host "Recording to: $RecordPath" -ForegroundColor Yellow
        }
        "headless" {
            # No display, no recording — just keep the port bound so a
            # separate Python process can pull from it. Useful for testing.
            Start-Process ffmpeg -ArgumentList "$ffArgs -f null -" -WindowStyle Hidden
        }
    }

    Start-Sleep -Milliseconds 1000

    # Tell camera to push
    Invoke-RestMethod "http://${CameraIP}:8080/gopro/camera/stream/start" | Out-Null
    Write-Host "Stream started. Camera pushing to udp://0.0.0.0:8554" -ForegroundColor Green
}

function Stop-GoProStream {
    [CmdletBinding()]
    param([string]$CameraIP = $Global:GoProCameraIP)
    try {
        Invoke-RestMethod "http://${CameraIP}:8080/gopro/camera/stream/stop" | Out-Null
    } catch { }
    Get-Process ffplay, ffmpeg -ErrorAction SilentlyContinue | Stop-Process -Force
    Write-Host "Stream stopped." -ForegroundColor Green
}

function Get-GoProStatus {
    [CmdletBinding()]
    param([string]$CameraIP = $Global:GoProCameraIP)
    $state = Invoke-RestMethod "http://${CameraIP}:8080/gopro/camera/state"
    [PSCustomObject]@{
        Battery        = $state.status.'70'
        SystemHot      = [bool]$state.status.'6'
        SystemBusy     = [bool]$state.status.'8'
        PreviewActive  = [bool]$state.status.'32'
        Mode           = $state.settings.'2'
        ResolutionCode = $state.settings.'3'
    }
}
