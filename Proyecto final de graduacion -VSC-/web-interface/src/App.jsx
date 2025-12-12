import React, { useState, useEffect, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';
import { Activity, Power, Crosshair, Zap, RotateCw } from 'lucide-react';
import './App.css';

function App() {
  const [port, setPort] = useState(null);
  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState("Desconectado");
  const [angleInput, setAngleInput] = useState("0");
  const [positionData, setPositionData] = useState([]);
  const [writer, setWriter] = useState(null);

  const [isFrozen, setIsFrozen] = useState(false);
  const isFrozenRef = useRef(false); // Ref para acceso dentro del loop serial

  const startTimeRef = useRef(null); // Ref para el tiempo de inicio del movimiento

  // Buffer for incoming serial data
  const bufferRef = useRef("");
  const readerRef = useRef(null);

  const connect = async () => {
    if (!navigator.serial) {
      alert("Tu navegador no soporta Web Serial API. Usa Chrome o Edge.");
      return;
    }

    try {
      const p = await navigator.serial.requestPort();
      await p.open({ baudRate: 115200 });
      setPort(p);
      setConnected(true);
      setStatus("Conectado");

      const textEncoder = new TextEncoderStream();
      const writableStreamClosed = textEncoder.readable.pipeTo(p.writable);
      const w = textEncoder.writable.getWriter();
      setWriter(w);

      const textDecoder = new TextDecoderStream();
      const readableStreamClosed = p.readable.pipeTo(textDecoder.writable);
      const r = textDecoder.readable.getReader();
      readerRef.current = r;

      // Start reading
      readLoop(r);
    } catch (err) {
      console.error(err);
      setStatus("Error: " + err.message);
      setConnected(false);
    }
  };

  const readLoop = async (r) => {
    try {
      while (true) {
        const { value, done } = await r.read();
        if (done) break;
        if (value) {
          bufferRef.current += value;
          processBuffer();
        }
      }
    } catch (error) {
      console.error(error);
      setStatus("Error de lectura");
    } finally {
      // Disconnected or closed
      setConnected(false);
    }
  };

  const processBuffer = () => {
    const lines = bufferRef.current.split('\n');
    bufferRef.current = lines.pop() || "";

    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed) continue;

      // console.log("RX:", trimmed); // Comentado para limpiar consola

      if (trimmed.startsWith("POS:")) {
        // Si está congelado (usando REF para valor fresco), ignoramos nuevos datos
        if (isFrozenRef.current) continue;

        const val = parseFloat(trimmed.substring(4));
        if (!isNaN(val)) {
          // Calcular tiempo transcurrido
          let timeLabel = "0.00";
          if (startTimeRef.current) {
            const diff = (Date.now() - startTimeRef.current) / 1000;
            timeLabel = diff.toFixed(2);
          }

          setPositionData(prev => {
            const newData = [...prev, { time: timeLabel, val }];
            if (newData.length > 500) return newData.slice(-500);
            return newData;
          });
        }
      } else if (trimmed.startsWith("STATUS:")) {
        const s = trimmed.substring(7);
        if (s === "JOGGING_LEFT") setStatus("Moviendo Izquierda...");
        else if (s === "JOGGING_RIGHT") setStatus("Moviendo Derecha...");
        else if (s === "STOPPED") setStatus("Detenido");
        else if (s === "HOME_SET") setStatus("Home Establecido");
        else if (s === "MOVING_TO") setStatus("Moviendo a objetivo...");
        else if (s === "MOVE_COMPLETE") setStatus("Movimiento completado");
        else setStatus(s);
      }
    }
  };

  const sendCommand = async (cmd) => {
    if (!writer) return;
    try {
      await writer.write(cmd + "\n");
      console.log("TX:", cmd);
    } catch (e) {
      console.error("Error writing:", e);
    }
  };

  const handleExecute = () => {
    const angle = parseFloat(angleInput);
    if (isNaN(angle)) {
      alert("Por favor ingrese un número válido.");
      return;
    }
    if (angle < -110 || angle > 110) {
      alert("¡ALERTA DE RANGO!\n\nEl ángulo debe estar entre -110° y 110°.\nValor ingresado: " + angle);
      return;
    }

    // Resetear para nuevo movimiento
    setPositionData([]);
    startTimeRef.current = Date.now();

    // Descongelar
    setIsFrozen(false);
    isFrozenRef.current = false;

    sendCommand("GOTO:" + angle);
  };

  const toggleFreeze = () => {
    const newState = !isFrozen;
    setIsFrozen(newState);
    isFrozenRef.current = newState; // Actualizar REF para el loop
  };

  const handleChartClick = (data) => {
    // Usamos la referencia para asegurar que leemos el valor actual sin depender del render
    if (!isFrozenRef.current) return;

    console.log("Chart Click Data:", data); // Para depuración en consola

    let clickedValue = null;

    // Caso A: Click desde un Dot específico (recharts suele enviar el payload directo)
    if (data && data.payload && data.payload.val !== undefined) {
      clickedValue = data.payload.val;
    }
    // Caso B: Click directo en el objeto de valor (algunos eventos de recharts)
    else if (data && data.val !== undefined) {
      clickedValue = data.val;
    }
    // Caso C: Click genérico en el chart (activePayload)
    else if (data && data.activePayload && data.activePayload.length > 0) {
      clickedValue = data.activePayload[0].payload.val;
    }

    if (clickedValue !== null) {
      setAngleInput(clickedValue.toString());
      setStatus("Seleccionado: " + clickedValue + "°"); // Feedback visual inmediato
    }
  };

  return (
    <div className="container">
      <div className="sidebar">
        {/* Connection Panel */}
        <div className="panel">
          <h2><Power size={18} style={{ marginRight: 8, verticalAlign: 'text-bottom' }} /> Conexión</h2>
          <div className="control-row">
            <span>Puerto Serial:</span>
            <button onClick={connect} disabled={connected}>
              {connected ? "CONECTADO" : "SELECCIONAR"}
            </button>
          </div>
          <div className="status-row">
            <div className={`led ${connected ? 'on' : 'off'}`}></div>
            <span>
              {connected ? "Conectado al dispositivo" : "Desconectado"}
              <br />
              <small style={{ color: '#888' }}>{status}</small>
            </span>
          </div>
        </div>

        {/* Homing Panel */}
        <div className="panel">
          <h2><Home size={18} style={{ marginRight: 8, verticalAlign: 'text-bottom' }} /> Homing</h2>
          <div className="jog-controls">
            <button className="jog-btn"
              onMouseDown={() => sendCommand("JOG_L")}
              onMouseUp={() => sendCommand("STOP_JOG")}
              onMouseLeave={() => sendCommand("STOP_JOG")}
            > &lt; Jog Izq </button>
            <button className="jog-btn"
              onMouseDown={() => sendCommand("JOG_R")}
              onMouseUp={() => sendCommand("STOP_JOG")}
              onMouseLeave={() => sendCommand("STOP_JOG")}
            > Jog Der &gt; </button>
          </div>
          <button className="btn-orange" onClick={() => sendCommand("SET_HOME")}>ESTABLECER HOME</button>
        </div>

        {/* Control Panel */}
        <div className="panel">
          <h2><Crosshair size={18} style={{ marginRight: 8, verticalAlign: 'text-bottom' }} /> Control</h2>
          <div className="control-row">
            <label>Ángulo Objetivo</label>
            <div style={{ display: 'flex', alignItems: 'center', gap: 5 }}>
              <input value={angleInput} onChange={e => setAngleInput(e.target.value)} type="number" step="0.1" />
              <span>°</span>
            </div>
          </div>
          <button className="btn-green" onClick={handleExecute}>
            EJECUTAR <Zap size={16} style={{ marginLeft: 8, verticalAlign: 'middle' }} />
          </button>
        </div>
      </div>

      <div className="main-content">
        <div className="chart-header">
          <h2><Activity size={20} style={{ marginRight: 10, verticalAlign: 'text-bottom' }} /> Posición en Tiempo Real</h2>
          <div style={{ display: 'flex', gap: 10 }}>
            <button
              onClick={toggleFreeze}
              style={{ padding: '5px 15px', fontSize: '0.8em', background: isFrozen ? '#e67e22' : '#333' }}>
              {isFrozen ? "REANUDAR GRÁFICA" : "CONGELAR GRÁFICA"}
            </button>
            <button
              onClick={() => setPositionData([])}
              style={{ padding: '5px 10px', fontSize: '0.8em', background: '#333' }}>
              Limpiar
            </button>
          </div>
        </div>
        <div className="chart-container">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={positionData}
              onClick={handleChartClick}
              style={{ cursor: isFrozen ? 'pointer' : 'default' }}
              margin={{ top: 5, right: 20, bottom: 5, left: 0 }}
            >
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="time"
                stroke="#666"
                fontSize={12}
                label={{ value: 'Tiempo (s)', position: 'insideBottomRight', offset: -5, fill: '#666' }}
              />
              <YAxis stroke="#666" fontSize={12} />
              <Tooltip
                wrapperStyle={{ pointerEvents: 'none' }}
                contentStyle={{ backgroundColor: '#222', borderColor: '#444', color: '#eee' }}
                itemStyle={{ color: '#00b894' }}
                formatter={(value) => [value + "°", "Ángulo"]}
                labelFormatter={(label) => "Tiempo: " + label + "s"}
              />
              <Line
                type="monotone"
                dataKey="val"
                stroke="#00b894"
                strokeWidth={2}
                dot={isFrozen}
                activeDot={{ r: 8 }}
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
    </div>
  )
}

function Home({ size, style }) { return <svg xmlns="http://www.w3.org/2000/svg" width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" style={style}><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"></path><polyline points="9 22 9 12 15 12 15 22"></polyline></svg> }

export default App;
