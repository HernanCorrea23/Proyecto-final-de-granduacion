import React, { useState, useEffect, useRef } from 'react';
import { Activity, Power, Crosshair, Zap, RotateCw, Save, Play, Square, Circle, Plus, Trash2, Edit2, Check, X, Target, BarChart2 } from 'lucide-react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import './App.css';

function App() {
  const [port, setPort] = useState(null);
  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState("Desconectado");
  const [writer, setWriter] = useState(null);
  const writerRef = useRef(null);

  const [mode, setMode] = useState("libre"); // "libre" o "grabacion"

  const [mPos, setMPos] = useState(0);
  const [sPos, setSPos] = useState(0);

  const [currentSequence, setCurrentSequence] = useState([]);
  const [figureName, setFigureName] = useState("");
  const [savedFigures, setSavedFigures] = useState(() => {
    const saved = localStorage.getItem("arm_figures");
    return saved ? JSON.parse(saved) : [];
  });

  const [editingFigureId, setEditingFigureId] = useState(null);
  const [editingName, setEditingName] = useState("");
  const [expandedFigureId, setExpandedFigureId] = useState(null);
  const [expandedChartId, setExpandedChartId] = useState(null);

  const readerRef = useRef(null);
  const bufferRef = useRef("");

  const executionState = useRef({
    running: false,
    mDone: false,
    sDone: false,
    waitServo: false,
    resolve: null
  });

  const offsetsRef = useRef({ m: 0, s: 0 });
  const rawMPosRef = useRef(0);
  const rawSPosRef = useRef(0);

  // Track keyboard state
  const keysPressed = useRef(new Set());
  const jogActive = useRef(false);

  // Connection
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
      textEncoder.readable.pipeTo(p.writable);
      const w = textEncoder.writable.getWriter();
      setWriter(w);
      writerRef.current = w;

      const textDecoder = new TextDecoderStream();
      p.readable.pipeTo(textDecoder.writable);
      const r = textDecoder.readable.getReader();
      readerRef.current = r;

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
      setConnected(false);
    }
  };

  const processBuffer = () => {
    const lines = bufferRef.current.split('\n');
    bufferRef.current = lines.pop() || "";

    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed) continue;

      if (trimmed.startsWith("M_POS:")) {
        const absM = parseInt(trimmed.substring(6));
        rawMPosRef.current = absM;
        setMPos(absM - offsetsRef.current.m);
      } else if (trimmed.includes("S_POS:")) {
        const parts = trimmed.split("S_POS:");
        const absS = parseInt(parts[1]);
        rawSPosRef.current = absS;
        setSPos(absS - offsetsRef.current.s);
      } else if (trimmed === "MDONE") {
        executionState.current.mDone = true;
        checkStepComplete();
      } else if (trimmed.includes("SDONE")) {
        executionState.current.sDone = true;
        checkStepComplete();
      } else if (trimmed.includes("SERVO_UP_OK") || trimmed.includes("SERVO_DOWN_OK")) {
        executionState.current.waitServo = true;
        checkStepComplete();
      }
    }
  };

  const checkStepComplete = () => {
    if (executionState.current.running) {
      if ((executionState.current.mDone && executionState.current.sDone) || executionState.current.waitServo) {
        if (executionState.current.resolve) {
          const res = executionState.current.resolve;
          executionState.current.resolve = null;
          res();
        }
      }
    }
  };

  const sendCommand = async (cmd) => {
    const w = writerRef.current || writer;
    if (!w) return;
    try {
      await w.write(cmd + "\n");
    } catch (e) {
      console.error("Error writing:", e);
    }
  };

  // Keyboard listeners
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return;
      if (e.target.tagName === 'INPUT') return;

      const k = e.key.toLowerCase();
      // Acciones discretas inmediatas
      if (['p', 'l', '+', '-'].includes(k)) {
        sendCommand(k);
        return;
      }

      keysPressed.current.add(k);
    };
    const handleKeyUp = (e) => {
      keysPressed.current.delete(e.key.toLowerCase());
    };
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  // Control loop (Jogging + Polling)
  useEffect(() => {
    const interval = setInterval(() => {
      if (!connected) return;

      // Disable manual jumping/polling if executing a figure
      if (executionState.current.running) return;

      const keys = keysPressed.current;
      let isJogging = false;

      // Handle continuous movement
      if (keys.has('a')) { sendCommand('a'); isJogging = true; }
      else if (keys.has('d')) { sendCommand('d'); isJogging = true; }
      else if (keys.has('w')) { sendCommand('w'); isJogging = true; }
      else if (keys.has('s')) { sendCommand('s'); isJogging = true; }

      if (!isJogging && jogActive.current) {
        sendCommand('x'); // Stop immediate
        jogActive.current = false;
      } else if (isJogging) {
        jogActive.current = true;
      }

      if (!isJogging) {
        window.pollTicks = (window.pollTicks || 0) + 1;
        if (window.pollTicks % 5 === 0) { // Every ~500ms
          sendCommand('c');
        }
      }

    }, 100);

    return () => clearInterval(interval);
  }, [connected]);

  // Save/Load effects
  useEffect(() => {
    localStorage.setItem("arm_figures", JSON.stringify(savedFigures));
  }, [savedFigures]);

  // Figure Execution
  const executeSequence = async (sequence) => {
    if (!connected || !writer) {
      alert("No hay conexión con el brazo.");
      return;
    }

    setStatus("Ejecutando figura...");
    executionState.current.running = true;

    for (let i = 0; i < sequence.length; i++) {
      const step = sequence[i];

      executionState.current.mDone = false;
      executionState.current.sDone = false;
      executionState.current.waitServo = false;

      setStatus(`Ejecutando punto ${i + 1}/${sequence.length}`);

      if (step.type === "motor") {
        const absM = step.m + offsetsRef.current.m;
        const absS = step.s + offsetsRef.current.s;
        sendCommand(`g${absM}`);
        sendCommand(`e${absS}`);

        await Promise.race([
          new Promise(resolve => executionState.current.resolve = resolve),
          new Promise(resolve => setTimeout(resolve, 15000))
        ]);

        await new Promise(r => setTimeout(r, 100)); // Small stabilization delay
      } else if (step.type === "servo") {
        sendCommand(step.val); // 'p' or 'l'
        await Promise.race([
          new Promise(resolve => executionState.current.resolve = resolve),
          new Promise(resolve => setTimeout(resolve, 2000))
        ]);
      }
    }

    executionState.current.running = false;
    setStatus("Figura completada.");
  };

  // UI Handlers
  const addMotorPoint = () => {
    setCurrentSequence([...currentSequence, { type: "motor", m: mPos, s: sPos }]);
  };

  const addServoPoint = (val, label) => {
    setCurrentSequence([...currentSequence, { type: "servo", val, label }]);
  };

  const clearSequence = () => setCurrentSequence([]);

  const saveFigure = () => {
    if (!figureName.trim() || currentSequence.length === 0) {
      alert("Ingrese un nombre y registre al menos un punto.");
      return;
    }
    setSavedFigures([...savedFigures, { id: Date.now(), name: figureName, points: currentSequence }]);
    setFigureName("");
    setCurrentSequence([]);
  };

  const deleteFigure = (id) => {
    setSavedFigures(savedFigures.filter(f => f.id !== id));
  };

  const startEditingFigure = (fig) => {
    setEditingFigureId(fig.id);
    setEditingName(fig.name);
  };

  const saveEditedFigureName = (id) => {
    if (!editingName.trim()) return;
    setSavedFigures(savedFigures.map(f =>
      f.id === id ? { ...f, name: editingName } : f
    ));
    setEditingFigureId(null);
  };

  const cancelEditingFigure = () => {
    setEditingFigureId(null);
  };

  const handleSetZero = () => {
    offsetsRef.current = { m: rawMPosRef.current, s: rawSPosRef.current };
    setMPos(0);
    setSPos(0);
  };

  return (
    <div className="dashboard">
      <header className="topbar">
        <div className="logo">
          <Activity className="text-neon" size={24} />
          <span>RoboDraw 2DoF</span>
        </div>
        <div className="topbar-controls">
          <div className={`status-badge ${connected ? 'status-on' : 'status-off'}`}>
            <div className="led"></div>
            {status}
          </div>
          <button className="btn-homing" onClick={() => sendCommand('h')} disabled={executionState.current.running || !connected}>
            <RotateCw size={18} /> Homing Automático
          </button>
          <button className="btn-connect" onClick={connect} disabled={connected}>
            <Power size={18} /> {connected ? "Conectado" : "Conectar Serial"}
          </button>
        </div>
      </header>

      <main className="content">
        <div className="panel modes-panel">
          <div className="tabs">
            <button className={`tab ${mode === 'libre' ? 'active' : ''}`} onClick={() => setMode('libre')}>Modo Libre</button>
            <button className={`tab ${mode === 'grabacion' ? 'active' : ''}`} onClick={() => setMode('grabacion')}>Modo Grabación</button>
          </div>

          <div className="tab-content">
            {mode === 'libre' ? (
              <div className="mode-libre">
                <h3><Crosshair size={20} /> Control de Teclado</h3>
                <p>Usa el teclado para mover el brazo robótico libremente. El movimiento es continuo mientras mantengas la tecla presionada.</p>
                <div className="key-grid">
                  <div className="key-item"><kbd>W</kbd> / <kbd>S</kbd><span>Motor Esclavo (Arriba/Abajo)</span></div>
                  <div className="key-item"><kbd>A</kbd> / <kbd>D</kbd><span>Motor Maestro (Izq/Der)</span></div>
                  <div className="key-item"><kbd>P</kbd> / <kbd>L</kbd><span>Servo (Elevar/Bajar Pen)</span></div>
                  <div className="key-item"><kbd>+</kbd> / <kbd>-</kbd><span>Servo Ajuste Fino</span></div>
                </div>
                <div className="live-pos" style={{ alignItems: 'stretch' }}>
                  <div className="pos-box">Maestro: <span>{mPos}</span> pasos</div>
                  <div className="pos-box">Esclavo: <span>{sPos}</span> pasos</div>
                  <button className="btn-zero" onClick={handleSetZero} title="Establecer origen en Cero (0, 0)">
                    <Target size={24} />
                  </button>
                </div>
              </div>
            ) : (
              <div className="mode-grabacion">
                <h3><Save size={20} /> Crear Figura</h3>
                <div className="live-pos" style={{ alignItems: 'stretch' }}>
                  <div className="pos-box">Maestro: <span>{mPos}</span></div>
                  <div className="pos-box">Esclavo: <span>{sPos}</span></div>
                  <button className="btn-zero" onClick={handleSetZero} title="Establecer origen en Cero (0, 0)">
                    <Target size={24} />
                  </button>
                </div>

                <div className="record-actions">
                  <button className="btn-record-motor" onClick={addMotorPoint}>
                    <Plus size={16} /> Registrar Motores (A,D,W,S)
                  </button>
                  <div className="servo-actions">
                    <button className="btn-record-servo" onClick={() => addServoPoint('p', 'Servo Arriba (P)')}>Registrar P (Arriba)</button>
                    <button className="btn-record-servo" onClick={() => addServoPoint('l', 'Servo Abajo (L)')}>Registrar L (Abajo)</button>
                  </div>
                </div>

                <div className="sequence-preview">
                  <h4>Puntos Registrados ({currentSequence.length})</h4>
                  <ul className="sequence-list">
                    {currentSequence.map((pt, i) => (
                      <li key={i}>
                        <span className="step-num">{i + 1}</span>
                        {pt.type === 'motor' ? `Ir a M:${pt.m}, S:${pt.s}` : `Acción: ${pt.label}`}
                      </li>
                    ))}
                    {currentSequence.length === 0 && <li className="empty-msg">No hay puntos aún. Mueve el eje libremente y registra posiciones.</li>}
                  </ul>
                  {currentSequence.length > 0 && <button className="btn-text" onClick={clearSequence}>Limpiar</button>}
                </div>

                <div className="save-form">
                  <input type="text" placeholder="Nombre de la figura (ej. Cuadrado)" value={figureName} onChange={e => setFigureName(e.target.value)} />
                  <button className="btn-save" onClick={saveFigure}><Save size={16} /> Guardar</button>
                </div>
              </div>
            )}
          </div>
        </div>

        <div className="panel figures-panel">
          <h3><Play size={20} /> Figuras Guardadas</h3>
          {savedFigures.length === 0 ? (
            <div className="empty-panel">
              <Circle className="icon-op" size={48} />
              <p>Aún no hay figuras guardadas. Crea una en el Modo Grabación.</p>
            </div>
          ) : (
            <div className="figures-grid">
              {savedFigures.map(fig => (
                <div className="figure-card-container" key={fig.id}>
                  <div className="figure-card" onDoubleClick={() => setExpandedFigureId(expandedFigureId === fig.id ? null : fig.id)}>
                    <div className="fig-info">
                      <Square size={24} className="text-neon" />
                      <div>
                        {editingFigureId === fig.id ? (
                          <div style={{ display: 'flex', gap: '5px', alignItems: 'center' }}>
                            <input
                              autoFocus
                              value={editingName}
                              onChange={e => setEditingName(e.target.value)}
                              onKeyDown={e => {
                                if (e.key === 'Enter') saveEditedFigureName(fig.id);
                                if (e.key === 'Escape') cancelEditingFigure();
                                e.stopPropagation();
                              }}
                              style={{ padding: '0.2rem 0.5rem', fontSize: '1rem', width: '130px', margin: 0 }}
                            />
                            <button style={{ padding: '0.3rem', background: 'rgba(0, 230, 118, 0.1)', color: '#00E676' }} onClick={() => saveEditedFigureName(fig.id)}><Check size={16} /></button>
                            <button style={{ padding: '0.3rem', background: 'rgba(255, 87, 34, 0.1)', color: '#FF5722' }} onClick={cancelEditingFigure}><X size={16} /></button>
                          </div>
                        ) : (
                          <>
                            <h4>{fig.name}</h4>
                            <small>{fig.points.length} puntos (doble clic para ver)</small>
                          </>
                        )}
                      </div>
                    </div>
                    <div className="fig-actions">
                      <button className="btn-play" onClick={() => executeSequence(fig.points)} disabled={!connected || executionState.current.running}>
                        <Play size={16} /> Ejecutar
                      </button>
                      <button style={{ background: 'rgba(0, 188, 212, 0.1)', color: '#00BCD4', padding: '0.6rem', border: '1px solid rgba(0, 188, 212, 0.3)' }} onClick={() => setExpandedChartId(expandedChartId === fig.id ? null : fig.id)} disabled={executionState.current.running} title="Ver Gráfica">
                        <BarChart2 size={16} />
                      </button>
                      <button style={{ background: 'rgba(255, 255, 255, 0.1)', color: '#8D99AE', padding: '0.6rem' }} onClick={() => startEditingFigure(fig)} disabled={executionState.current.running}>
                        <Edit2 size={16} />
                      </button>
                      <button className="btn-del" onClick={() => deleteFigure(fig.id)} disabled={executionState.current.running}><Trash2 size={16} /></button>
                    </div>
                  </div>
                  {expandedFigureId === fig.id && (
                    <div className="figure-details">
                      <ul className="sequence-list">
                        {fig.points.map((pt, i) => (
                          <li key={i}>
                            <span className="step-num">{i + 1}</span>
                            {pt.type === 'motor' ? `M: ${pt.m}, S: ${pt.s}` : `Acción: ${pt.label}`}
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                  {expandedChartId === fig.id && (
                    <div className="figure-chart-details">
                      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: "1rem" }}>
                        <h4 style={{ color: "#fff", fontSize: "0.95rem", margin: 0 }}>Movimiento de Motores</h4>
                        <span style={{ color: "var(--neon-green)", fontSize: "0.85rem", background: "rgba(0, 230, 118, 0.1)", padding: "0.3rem 0.8rem", borderRadius: "12px", display: "flex", alignItems: "center", gap: "0.3rem" }}>
                          ⏱️ Tiempo Est.: ~{
                            (fig.points.reduce((acc, curr, i, arr) => {
                              if (i === 0) return acc + 0.5;
                              if (curr.type === 'servo') return acc + 1.5;

                              const prevMotors = arr.slice(0, i).reverse().find(p => p.type === 'motor');
                              if (prevMotors && curr.type === 'motor') {
                                const distM = Math.abs(curr.m - prevMotors.m);
                                const distS = Math.abs(curr.s - prevMotors.s);
                                // Estimación: ~15000 pasos/segundo en crucero, más ~0.3s por rampas de aceleración
                                return acc + (Math.max(distM, distS) / 15000) + 0.3;
                              }
                              return acc + 1;
                            }, 0)).toFixed(1)
                          } s
                        </span>
                      </div>
                      <ResponsiveContainer width="100%" height={320}>
                        <LineChart margin={{ top: 20, right: 30, left: 20, bottom: 65 }} data={fig.points.filter(p => p.type === 'motor').map((p, i) => ({ paso: i + 1, Maestro: p.m, Esclavo: p.s }))}>
                          <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.1)" />
                          <XAxis dataKey="paso" stroke="#8D99AE" tick={{ fill: '#8D99AE' }} label={{ value: 'Punto Registrado N°', position: 'bottom', offset: 15, fill: '#8D99AE', fontSize: 13 }} />
                          <YAxis width={80} stroke="#8D99AE" tick={{ fill: '#8D99AE' }} label={{ value: 'Posición (Pasos)', angle: -90, position: 'insideLeft', offset: -5, fill: '#8D99AE', fontSize: 13 }} />
                          <Tooltip contentStyle={{ backgroundColor: 'rgba(0,0,0,0.8)', border: '1px solid #00F0FF', borderRadius: '8px', color: '#fff' }} />
                          <Legend verticalAlign="bottom" wrapperStyle={{ paddingTop: "40px" }} />
                          <Line type="monotone" dataKey="Maestro" stroke="#00F0FF" strokeWidth={2} dot={{ r: 4, fill: '#00F0FF' }} activeDot={{ r: 8 }} />
                          <Line type="monotone" dataKey="Esclavo" stroke="#FF00A0" strokeWidth={2} dot={{ r: 4, fill: '#FF00A0' }} />
                        </LineChart>
                      </ResponsiveContainer>
                    </div>
                  )}
                </div>
              ))}
            </div>
          )}
        </div>
      </main>
    </div>
  );
}

export default App;