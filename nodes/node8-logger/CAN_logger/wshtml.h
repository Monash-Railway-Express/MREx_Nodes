const String wshtml = R"wshtml(
<!DOCTYPE html>
<html lang="en">
    <head>
        <script>
            window.onload = function () {
                const log = document.getElementById("log");
                const rawLog = document.createElement("div");

                log.replaceChildren(rawLog);

                if (window.EventSource) {
                    let conn;

                    document.getElementById("connect").onclick = function (evt) {
                        if (conn) {
                            conn.close();
                        }
                        
                        conn = new EventSource(document.getElementById("sseURL").value);

                        conn.onmessage = function (evt) {
                            const messages = evt.data.split('\n');
                            for (let i = 0; i < messages.length; i++) {
                                const item = document.createElement("pre");
                                item.innerText = messages[i];
                                rawLog.appendChild(item);
                            }
                        };
                    }
                } else {
                    const item = document.createElement("pre");
                    item.innerHTML = "<b>Your browser does not support Server-Sent Events.</b>";
                    rawLog.appendChild(item);
                }
            };
        </script>
    </head>
    <body>
        <input type="button" id="connect" value="Connect">
        <input type="text" id="sseURL" value="http://10.0.0.8/serial">
        <div id="log"></div>
    </body>
</html>
)wshtml";