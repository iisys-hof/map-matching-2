[string]$Program = ".\run\msvc\release\bin\map_matching_2.exe"

function Run-Test {
    param (
        [string]$FolderName,
        [double]$Accuracy,
        [bool]$CleanUp
    )

    & $Program --conf "data\$FolderName\conf\prepare.conf" --memory-mapped-preparation off | Out-Null
    & $Program --conf "data\$FolderName\conf\match.conf" | Out-Null
    $output = & $Program --conf "data\$FolderName\conf\compare.conf" | Out-String

    if ($output -match 'weighted mean correct fraction:\s*([0-9.]+)') {
        $value = [double]$matches[1]
        if ($value -ge $Accuracy) {
            Write-Host "$FolderName $value >= $Accuracy PASS"
        }
        else {
            Write-Host "$FolderName $value >= $Accuracy FAIL"
        }
    }
    else {
        Write-Host "ERROR: Could not parse output"
    }

    if ($CleanUp -and (Test-Path "data\$FolderName\network")) {
        Remove-Item -Path "data\$FolderName\network" -Recurse -Force
    }
}

Run-Test "floating-car-data" 0.99701 $true
Run-Test "gis-cup" 0.98593 $true
Run-Test "hengfeng-li" 0.99803 $true
Run-Test "kubicka-et-al" 0.98736 $true
Run-Test "newson-krumm" 0.99889 $true
