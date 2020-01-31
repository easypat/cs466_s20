# re is a quick function ro reload 
define re
    monitor reset halt
    load
    info threads
    monitor reset init
end

define reload
    re
end


target extended-remote :3333
set print elements 350
set confirm off
set pagination off

re
b main

