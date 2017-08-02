$WorkPath = "E:\点云配准\Hive\branches\Point_Matching\PointCloudRegistration"
$ReplaceWorkPath = "E:\点云配准\Hive\branches\Point_Matching\PointCloudRegistration\\"
Set-Location $WorkPath
$Files = "TestData\output\New.ply","TestData\output\cost.txt","TestData\output\RegMsg.txt","RegCfg.txt"


function Pro-Copy($StrPath, $FileName)
{
    Get-ChildItem 'TestData\output\TmpPro' | Remove-Item
    Copy-Item ($StrPath + 'Gss.ply') -Destination 'TestData\output\TmpPro\Tmp_Gss.ply'
    Copy-Item ($StrPath + 'K1.ply') -Destination 'TestData\output\TmpPro\Tmp_K1.ply'
    Copy-Item ($StrPath + 'K2.ply') -Destination 'TestData\output\TmpPro\Tmp_K2.ply'
    Copy-Item ($StrPath + 'Mean.ply') -Destination 'TestData\output\TmpPro\Tmp_Mean.ply'
    Copy-Item $FileName -Destination 'TestData\output\TmpPro\Tmp_Norm.ply'
}

function Coarse-Copy($NewDir, $CoarseFilePath)
{
#    $Null = New-Item -Name ($NewDir -replace $ReplaceWorkPath) -type directory -ErrorAction SilentlyContinue 
    $Null = New-Item -Name $NewDir -type directory -ErrorAction SilentlyContinue 
    Get-ChildItem $NewDir | Remove-Item
    $DirectLeaf = Split-Path -Leaf $NewDir
    $ParentLeaf = Split-Path ((Split-Path -Parent $NewDir)) -Leaf
    $Files | ForEach-Object { Copy-Item $_ -Destination $NewDir }
    Get-ChildItem $NewDir | Rename-Item -NewName { 
        $Extension = [System.IO.Path]::GetExtension($_.Name)
        'c_{0}_{1}_{2}{3}' -f $ParentLeaf, $DirectLeaf,$_.BaseName,$Extension
    }

    Pro-Copy $CoarseFilePath "TestData\output\New.ply"
}

function Fine-Copy($NewDir)
{
#    $Null = New-Item -Name ($NewDir -replace $ReplaceWorkPath) -type directory -ErrorAction SilentlyContinue 
    $Null = New-Item -Name $NewDir -type directory -ErrorAction SilentlyContinue 
    Get-ChildItem $NewDir | Remove-Item
    $DirectLeaf = Split-Path -Leaf $NewDir
    $ParentLeaf = Split-Path ((Split-Path -Parent $NewDir)) -Leaf
    $Files | ForEach-Object { Copy-Item $_ -Destination $NewDir }
    Get-ChildItem $NewDir | Rename-Item -NewName { 
        $Extension = [System.IO.Path]::GetExtension($_.Name)
        'f_{0}_{1}_{2}{3}' -f $ParentLeaf, $DirectLeaf,$_.BaseName,$Extension
    }
	
	
	Get-ChildItem 'TestData\output\LastTmp' | Remove-Item
    Copy-Item 'TestData\output\TmpPro\Tmp_Gss.ply' -Destination 'TestData\output\LastTmp\Last_Gss.ply'
    Copy-Item 'TestData\output\TmpPro\Tmp_K1.ply' -Destination 'TestData\output\LastTmp\Last_K1.ply'
    Copy-Item 'TestData\output\TmpPro\Tmp_K2.ply' -Destination 'TestData\output\LastTmp\Last_K2.ply'
    Copy-Item 'TestData\output\TmpPro\Tmp_Mean.ply' -Destination 'TestData\output\LastTmp\Last_Mean.ply'
    Copy-Item 'TestData\output\TmpPro\Tmp_Norm.ply' -Destination 'TestData\output\LastTmp\Last_Norm.ply'
}