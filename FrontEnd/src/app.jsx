import { styled } from "styled-components"
import { useContext, useEffect, useState } from "preact/hooks"
import { AnimatePresence, motion } from "framer-motion"
import Connecting from "./Screens/Connecting"
import FlightData from "./Screens/FlightData"
import AppContext from "./context/data"
import Div100vh from 'react-div-100vh'

export function App() {
  const { socketStatus } = useContext(AppContext)
  const [socketConnected, setSocketConnected] = useState(false)
  useEffect(() => {
    const timer = setTimeout(() => {
      setSocketConnected(socketStatus == "Connected")
    }, 600)
    return () => {
      clearTimeout(timer)
    }
  }, [socketStatus])


  return (
    <MobileContainer>
      <AnimatePresence mode="popLayout">
        {!socketConnected ?
          <ContentBox
            initial={{ y: "-100%" }}
            animate={{ y: 0 }}
            exit={{ y: "-100%", zIndex: 2 }}
            key="ConnectingBox"
            transition={{
              duration: 0.5,
              ease: "easeInOut",
            }}
          >
            <Connecting />
          </ContentBox>
          :
          <ContentBox
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            key="FlightBox"
          >
            <FlightData />
          </ContentBox>
        }
      </AnimatePresence>
    </MobileContainer>
  )
}

const MobileContainer = styled(Div100vh)`
  width: 100vw;
  max-width: 512px;
  /* aspect-ratio: 9/16; */
  display: flex;
  flex-direction: column;
  align-items: center;
  background-color: var(--bgColorDark);
  overflow: hidden;
  `
const ContentBox = styled(motion.div)`
  width: 100%;
  height: 100%;
  display: grid;
  place-items: center;
  overflow: hidden;

`