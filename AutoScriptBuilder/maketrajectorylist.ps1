$BotName = "SwervyBot"
"let trajectoryfiles = $((Get-ChildItem ..\$BotName\src\main\deploy\pathplanner\*.path).basename | ConvertTo-Json )"  > trajectoryfilelist.js