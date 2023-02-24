pushd
Set-Location $PSScriptRoot
$BotName = "Clarke"
"let trajectoryfiles = $((Get-ChildItem ..\$BotName\src\main\deploy\pathplanner\*.path).basename | ConvertTo-Json )"  > trajectoryfilelist.js
"let savedRobotPoses = $(select-string 'Map.entry.*new Pose2d' ..\$BotName\src\main\java\frc\robot\Constants.java | Foreach-Object {$_.line.split('"')[1]} | ConvertTo-Json)" > savedRobotPoses.js
popd