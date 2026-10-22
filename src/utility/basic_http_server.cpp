#include "basic_http_server.hpp"
#include <QMap>
#include <QTcpSocket>
#include <QStringList>
#include <QDateTime>
#include <QVariantList>
#include <QTimer>
#include <iostream>
//#include "camera_window.hpp"
#include "json.hpp"

namespace bias
{
    // Constants
    // ------------------------------------------------------------------------
    static QMap<QString,QString> createEscapeToCharMap()
    {
        QMap<QString,QString> map;
        map[QString("%20")] = QString(" ");
        map[QString("%24")] = QString("$");
        map[QString("%26")] = QString("&");
        map[QString("%60")] = QString("`");
        map[QString("%3A")] = QString(":");
        map[QString("%3C")] = QString("<");
        map[QString("%3E")] = QString(">");
        map[QString("%5B")] = QString("[");
        map[QString("%5D")] = QString("]");
        map[QString("%7B")] = QString("{");
        map[QString("%7D")] = QString("}");
        map[QString("%22")] = QString("\"");
        map[QString("%23")] = QString("#");
        map[QString("%25")] = QString("%");
        map[QString("%40")] = QString("@");
        map[QString("%2F")] = QString("/");
        map[QString("%3B")] = QString(";");
        map[QString("%3D")] = QString("=");
        map[QString("%3F")] = QString("?");
        map[QString("%5C")] = QString("\\"); 
        map[QString("%5E")] = QString("^");
        map[QString("%7C")] = QString("|");
        map[QString("%7E")] = QString("~"); 
        map[QString("%27")] = QString("'");
        map[QString("%5C")] = QString("\\"); 
        map[QString("%2C")] = QString(",");
        return map;
    }
    QMap<QString,QString> ESCAPE_TO_CHAR_MAP = createEscapeToCharMap();

    // Close a kept-open (keep-alive) connection after this many ms of inactivity, so a
    // client that vanishes without closing the socket doesn't leak a connection. Re-armed
    // on every request; far longer than the gap between polls at any sane rate.
    const int IDLE_TIMEOUT_MS = 5000;


    // Methods - public
    // -------------------------------------------------------------------------
    BasicHttpServer::BasicHttpServer(QObject *parent)
        : QTcpServer(parent)
    { 
    }

    void BasicHttpServer::incomingConnection(qintptr socket) 
    { 
        QTcpSocket* s = new QTcpSocket(this);
        connect(s, SIGNAL(readyRead()), this, SLOT(readClient()));
        connect(s, SIGNAL(disconnected()), this, SLOT(discardClient()));

        // Idle-timeout watchdog: closes the connection if it goes quiet (handles a
        // keep-alive client that disappears without closing, and a client that connects
        // but never sends a complete request). The timer is a child of the socket, so it
        // is destroyed with it. Armed now and re-armed on each request in readClient().
        QTimer* idleTimer = new QTimer(s);
        idleTimer->setSingleShot(true);
        connect(idleTimer, &QTimer::timeout, s, &QTcpSocket::close);
        idleTimer->start(IDLE_TIMEOUT_MS);

        s->setSocketDescriptor(socket);
    }


    // Protected methods
    // ------------------------------------------------------------------------
    bool BasicHttpServer::handleGetRequest(QTcpSocket *socketPtr, QStringList &tokens)
    {
        QTextStream os(socketPtr);
        os.setAutoDetectUnicode(true);

        // Examine tokens
        if (tokens.size() < 2)
        {
            sendBadRequestResp(os,"not enought tokens");
            return false;
        }

        // Parse tokens
        QString paramsString = tokens[1];
        paramsString = replaceEscapeChars(paramsString);

        if (paramsString.length() == 1)
        {
            sendRunningResp(os);
            return false;
        }
        else if (paramsString.length() > 1)
        {
            // Check for request parameters character '?'
            QChar secondChar = paramsString[1];
            if (secondChar != QChar('?'))
            {
                sendBadRequestResp(os, "no ? character preceeding parameters");
                return false;
            }

            paramsString.remove(0,2);
            QStringList paramsList = paramsString.split("&",QString::SkipEmptyParts);
            if (!paramsList.isEmpty())
            {
                // We have some parameters - send appropriate response. The params
                // path writes its own response (with Content-Length) directly to the
                // socket so we do not use the QTextStream 'os' here.
                return handleParamsRequest(socketPtr, paramsList);
            }
            else
            {
                // No parameters follow '?' character
                sendBadRequestResp(os,"not parameters following ? char");
                return false;
            }
        }
        return false;
    }


    bool BasicHttpServer::handleParamsRequest(QTcpSocket *socketPtr, QStringList &paramsList)
    {
        // Handle requests. A connection is kept open (HTTP keep-alive) only when the
        // client explicitly opts in with a "keep-alive=1" argument AND every command in
        // the request is "plugin-cmd". Any other command forces the connection closed,
        // so keep-alive is impossible for non-polling commands.
        QVariantList respList;
        QVariantMap cmdMap;
        bool keepAliveRequested = false;
        bool allPluginCmd = true;
        int numCmd = 0;
        for (int i=0; i<paramsList.size(); i++)
        {
            QStringList parts = paramsList[i].split("=",QString::SkipEmptyParts);
            if (parts.size() == 0)
            {
                // Nothing here - just skip it
                continue;
            }
            QString name = parts[0];
            QString value = (parts.size() >= 2) ? parts[1] : QString("");

            if (name == QString("keep-alive"))
            {
                // Transport control argument, not a command - consume it here.
                keepAliveRequested = (value == QString("1")) ||
                                     (value.toLower() == QString("true"));
                continue;
            }

            numCmd++;
            if (name != QString("plugin-cmd"))
            {
                allPluginCmd = false;
            }

            if (parts.size() <= 2)
            {
                // Command name (+ optional parameters)
                cmdMap = paramsRequestSwitchYard(name, value);
            }
            else
            {
                // Error unable to parse command
                cmdMap = QVariantMap();
                cmdMap.insert("success", false);
                cmdMap.insert("message", "unable to parse command");
            }
            respList.append(cmdMap);
        }

        bool keepAlive = keepAliveRequested && allPluginCmd && (numCmd > 0);

        // Serialize the body first so we can send a Content-Length header. A reused
        // (keep-alive) connection needs Content-Length to find where each response
        // ends; close-style clients ignore it harmlessly.
        bool ok;
        QByteArray jsonResp = QtJson::serialize(respList,ok);

        QByteArray response;
        response += "HTTP/1.0 200 Ok\r\n";
        response += "Content-Type: application/json; charset=\"utf-8\"\r\n";
        response += "Content-Length: " + QByteArray::number(jsonResp.size()) + "\r\n";
        response += keepAlive ? "Connection: keep-alive\r\n" : "Connection: close\r\n";
        response += "\r\n";
        response += jsonResp;

        socketPtr->write(response);
        socketPtr->flush();
        return keepAlive;
    }


    void BasicHttpServer::sendBadRequestResp(QTextStream &os, QString msg)
    { 
        os << "HTTP/1.0 400 Bad Request\r\n";
        os << "Content-Type: text/html; charset=\"utf-8\"\r\n\r\n";
        os << "<html>\n";
        os << "<body>\n";
        os << "<h1>BIAS External Control Server</h1>\n";
        os << "Bad request: " << msg << "\n";
        os << "</body>\n";
        os << "</html>\n";
    }


    void BasicHttpServer::sendRunningResp(QTextStream &os)
    { 
        os << "HTTP/1.0 200 Ok\r\n";
        os << "Content-Type: text/html; charset=\"utf-8\"\r\n\r\n";
        os << "<html>\n";
        os << "<body>\n";
        os << "<h1>BIAS Server Running</h1>\n";
        os << QDateTime::currentDateTime().toString() << "\n";
        os << "</body>\n";
        os << "</html>\n";
    }


    QVariantMap BasicHttpServer::paramsRequestSwitchYard(QString name, QString value)
    {
        QVariantMap cmdMap;
        cmdMap.insert("command", name);
        cmdMap.insert("success", true);
        cmdMap.insert("message", "test response");
        cmdMap.insert("value", "hello world");
        return cmdMap;
    }


    // Protected slots
    // ------------------------------------------------------------------------
    void BasicHttpServer::readClient()
    {
        QTcpSocket* socketPtr = (QTcpSocket*) sender();

        // Process every complete request currently buffered on the socket. A GET
        // request is complete once the blank line terminating its headers has been
        // received (GET has no body). Looping lets a single keep-alive connection
        // serve back-to-back requests; the connection stays open between them unless
        // a request declines keep-alive (or is not an all-plugin-cmd request).
        while (socketPtr->bytesAvailable() > 0)
        {
            // Wait until the full header block has arrived so draining never stops
            // mid-request (which would desync the next request on a kept-open socket).
            QByteArray buffered = socketPtr->peek(socketPtr->bytesAvailable());
            if (!buffered.contains("\r\n\r\n") && !buffered.contains("\n\n"))
            {
                return; // headers not fully received yet - wait for more data
            }

            // Read the request line, then drain the remaining header lines.
            QString requestString = QString(socketPtr->readLine());
            while (socketPtr->canReadLine())
            {
                QByteArray line = socketPtr->readLine();
                if (line == "\r\n" || line == "\n")
                {
                    break; // end of headers
                }
            }

            bool keepAlive = false;
            QStringList tokens = splitRequestString(requestString);
            if (!tokens.isEmpty() && tokens[0] == "GET")
            {
                keepAlive = handleGetRequest(socketPtr, tokens);
            }

            if (!keepAlive)
            {
                socketPtr -> close();
                if (socketPtr -> state() == QTcpSocket::UnconnectedState)
                {
                    socketPtr -> deleteLater();
                }
                return;
            }

            // Connection kept open - reset the idle-timeout watchdog.
            QTimer* idleTimer = socketPtr->findChild<QTimer*>();
            if (idleTimer != nullptr)
            {
                idleTimer->start(IDLE_TIMEOUT_MS);
            }
        }
    }


    void BasicHttpServer::discardClient()
    {
        QTcpSocket* socketPtr = (QTcpSocket*)sender(); 
        socketPtr->deleteLater();
    }


    // Utility functions
    // ------------------------------------------------------------------------
    QStringList splitRequestString(QString reqString)
    {
        QStringList reqList;
        if (reqString.isEmpty())
        {
            return reqList;
        }
        int len = reqString.length();
        int n0 = reqString.indexOf(' ');
        int n1 = reqString.lastIndexOf(' ');
        if ((n0 == -1) || (n1 == -1) || (n0==n1))
        {
            return reqList;
        }
        QString token0 = reqString.left(n0); 
        QString token1 = reqString.mid(n0+1,n1-n0-1).trimmed();
        QString token2 = reqString.right(len-n1-1);
        reqList.append(token0);
        reqList.append(token1);
        reqList.append(token2);
        return reqList;
    }

    QString replaceEscapeChars(QString input)
    {
        QString output(input);
        QMap<QString,QString>::iterator it;
        for (it=ESCAPE_TO_CHAR_MAP.begin(); it!=ESCAPE_TO_CHAR_MAP.end(); it++)
        {
            output.replace(it.key(), it.value());
        }
        return output;
    }

} // namespace bias
