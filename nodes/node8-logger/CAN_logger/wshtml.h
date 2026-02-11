const String wshtml = R"wshtml(
<!DOCTYPE html>
<html lang="en">
<head>
    <title>Web Socket Client Example</title>
    <script type="text/javascript">
        window.onload = function () {
            var conn;
            var log = document.getElementById("log");
            var msg = document.getElementById("msg");

            function appendLog(item) {
                var doScroll = log.scrollTop === log.scrollHeight - log.clientHeight;
                log.appendChild(item);
                if (doScroll) {
                    log.scrollTop = log.scrollHeight - log.clientHeight;
                }
            }

            conn = new WebSocket("ws://10.0.0.1/ws");
            if (window["WebSocket"]) {
                if (conn) {
                    conn.onopen = function (evt) {
                        document.getElementById("sendMsg").disabled = false
                        document.getElementById("status").innerHTML = "Connection opened"
                    }
                    conn.onclose = function (evt) {
                        document.getElementById("status").innerHTML = "Connection closed"
                    };
                    conn.onmessage = function (evt) {
                        var messages = evt.data.split('\n');
                        for (var i = 0; i < messages.length; i++) {
                            var item = document.createElement("pre");
                            item.innerText = messages[i];
                            appendLog(item);
                        }
                    }
                }
            } else {
                var item = document.createElement("pre");
                item.innerHTML = "<b>Your browser does not support WebSockets.</b>";
                appendLog(item);
            }

            document.getElementById("form").onsubmit = function () {
                if (!conn) {
                    return false;
                }
                if (!msg.value) {
                    return false;
                }
                conn.send(msg.value);
                var item = document.createElement("pre");
                item.classList.add("subscribeMsg");
                item.innerHTML = msg.value;
                appendLog(item);
                return false;
            };
        };

    </script>
    <style type="text/css">
        html {
            overflow: hidden;
        }

        body {
            overflow: hidden;
            padding: 0;
            margin: 0;
            width: 100%;
            height: 100%;
            background: gray;
        }

        #log {
            background: white;
            margin: 0;
            padding: 0.5em 0.5em 0.5em 0.5em;
            top: 1.5em;
            left: 0.5em;
            right: 0.5em;
            bottom: 3em;
            overflow: auto;
            height: 530px;
        }

        #form {
            padding: 0 0.5em 0 0.5em;
            margin: 0;
            bottom: 3em;
            top: 5em;
            left: 8px;
            width: 100%;
            overflow: hidden;
        }

        #serverLocation {
            padding-top: 0.3em;
        }

        #requestSection {
            height: 38px;
        }

        #responseMsgSection {
            height: 570px;
            position: relative;
        }
    </style>
</head>
<body>
<fieldset>
    <legend>Server Status</legend>
    <div>
        <span id="status"></span>
    </div>
</fieldset>
<fieldset>
    <legend>Request</legend>
    <form id="form">
        <input type="submit" value="Send" id="sendMsg" disabled/>
        <input type="text" size="80" id="msg"/>
    </form>
</fieldset>
<fieldset>
    <legend>Response</legend>
    <div id="log"></div>
</fieldset>
</body>
</html>
)wshtml";