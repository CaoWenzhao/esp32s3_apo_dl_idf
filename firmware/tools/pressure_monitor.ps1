param(
    [string]$Port = "COM17",
    [int]$BaudRate = 115200
)

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    $Host.UI.RawUI.WindowTitle = "IMS032 pressure monitor - $Port"
    Write-Host "IMS032-S40A pressure monitor on $Port @ $BaudRate baud" -ForegroundColor Cyan
    Write-Host "Press Ctrl+C to stop.`n" -ForegroundColor DarkGray

    while ($true) {
        try {
            $line = $serial.ReadLine()
            if ($line -match "pressure:\s+(.*)$") {
                Write-Host ("{0:HH:mm:ss.fff}  {1}" -f (Get-Date), $Matches[1])
            }
        } catch [System.TimeoutException] {
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
