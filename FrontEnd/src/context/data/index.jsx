import { createContext } from "preact";
import { useEffect, useRef, useState } from "preact/hooks"

const AppContext = createContext({
  socketStatus: 'Connecting',
  status: "Initializing",
  data: {},
  socketConnected: false,
  apogee: 0,
  fatalError: null,
  log: [],
  battery: 0,
  altitude_offset: 0,
  sea_level_pressure: 0
});

const URL = `ws://${location.host}/ws`;

const AppContextProvider = ({ children }) => {
  const [socketConnected, setSocketConnected] = useState(false)
  const [data, setData] = useState({})
  const [status, setStatus] = useState("Connecting")
  const [apogee, setApogee] = useState(0)
  const [log, setlog] = useState([])
  const [battery, setBattery] = useState(0)
  const [altitude_offset, setAltitude_offset] = useState(0)
  const [sea_level_pressure, setSea_level_pressure] = useState(0)
  const [fatalError, setFaltaError] = useState({})
  const socket = useRef(null)


  function connect_socket() {

    if (socket.current) {
      socket.current.close()
      socket.current = null;
    }
    setSocketConnected(false)

    socket.current = new WebSocket(URL);

    socket.current.onopen = () => setSocketConnected(true);
    socket.current.onclose = () => setSocketConnected(false);

    socket.current.onmessage = (event) => {
      if (JSON.parse(event.data)) {
        let parsed = (JSON.parse(event.data))
        if (parsed.sensors) {
          setData((p) => ({ ...p, ...parsed.sensors }));
        }
        if (parsed.status) {
          setStatus(parsed.status)
        }

        if (parsed.apogee) {
          setApogee(parsed.apogee)
        }

        if (parsed.fatalError) {
          setFaltaError(parsed.fatalError)
        }

        if (parsed.log) {
          setlog(old => [...old, parsed.log])
        }

        if (parsed.battery) {
          setBattery(parsed.battery)
        }
        if (parsed.offset_altitude) {
          setAltitude_offset(parsed.offset_altitude)
        }
        if (parsed.sea_level_pressure) {
          setSea_level_pressure(parsed.sea_level_pressure)
        }
      }
    }
    return () => {
      socket.current.close()
    }
  }


  useEffect(() => {
    const Timer = setTimeout(() => {
      if (socket && socketConnected) {
        console.log("Connection Off")
        setSocketConnected(false);
      }
    }, 3000)
    return () => {
      clearTimeout(Timer)
    }
  }, [data])

  const reconnectTimer = useRef()

  useEffect(() => {
    if (!socketConnected) {
      let timer = setInterval(() => {
        console.log("Reconnecting")
        connect_socket();
      }, 3000)
      reconnectTimer.current = timer;
      return () => {
        clearInterval(reconnectTimer.current)
      }
    } else {
      if (reconnectTimer.current) {
        clearInterval(reconnectTimer.current)
      }
    }
  }, [socketConnected])

  useEffect(() => {
    if (!socket.current) {
      connect_socket()
    }
  }, [socket?.current])

  const socketStatus = {
    [0]: 'Connecting',
    [1]: 'Connected',
    [2]: 'Closing',
    [3]: 'Closed',
    [4]: 'Uninstantiated',
  }[socket?.current?.readyState ?? 0];

  return (
    <AppContext.Provider value={{
      status: (!socketConnected ? "Connecting" : status),
      socketStatus,//: !socketConnected ? "Connecting" : socketStatus,
      data,
      socketConnected,
      apogee,
      fatalError,
      log,
      battery,
      altitude_offset,
      sea_level_pressure,
    }}>
      {children}
    </AppContext.Provider >
  );
}

export default AppContext;
export { AppContextProvider };