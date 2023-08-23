import { styled } from "styled-components";
import { ReactComponent as SvgLogo } from "../../assets/logo.svg";
import { ReactComponent as SvgPatch } from "../../assets/patch.svg";
import { ReactComponent as ArrowUp } from "../../assets/arrow_up.svg";
import { ReactComponent as ArrowDown } from "../../assets/arrow_down.svg";
import { ReactComponent as BatteryFull } from "../../assets/battery_full.svg";
import { ReactComponent as BatteryHalf } from "../../assets/battery_half.svg";
import { ReactComponent as BatteryEmpty } from "../../assets/battery_empty.svg";

import { ReactComponent as XSvg } from "../../assets/x.svg";
import { ReactComponent as ISvg } from "../../assets/i.svg";
import { motion } from "framer-motion";
import AppContext from "../../context/data";
import { useContext, useState } from "preact/hooks";

function FlightData() {
  const { status, data, apogee, fatalError, log, battery, sea_level_pressure, altitude_offset } = useContext(AppContext)

  const [openLogs, setOpenLogs] = useState(false)
  const [logLenght, setlogLenght] = useState(0)

  const statusColor = {
    "Initializing": "#006eff",
    "Ready": "#00ff00",
    "Rising": "#00f7ff",
    "Recovering": "#ffee00",
    "Falling": "#ff8800",
    "Error": "#ff0000",
  }
  return (
    <FlightDataContainer>
      <StatusBox >
        <Patch />
        <Status
          animate={{
            borderBottom: `0.3rem solid ${statusColor[status] ?? "#ff0000"}`,
          }}
        >
          <span>{status}</span>
        </Status>
      </StatusBox>
      <DataBox>
        {Object.entries(data).map(([key, info], index) => {
          return <DataInfo key={key} info={info} startOrder={info?.order ? Object.keys(data).length - info?.order : Object.keys(data).length - index} />
        })}
      </DataBox>
      <ApogeeInfo
        initial={{
          x: "-100%",
        }}
        animate={{
          x: apogee > 1.0 ? 0 : "-100%",
        }}
      >
        <span>Apogeu:</span>
        <b>{apogee?.toFixed(2) ?? 0} m</b>
      </ApogeeInfo>

      <LogoBox >
        <Logo />
        <span>
          APEX <b>ROCKETRY</b>
        </span>
      </LogoBox>
      {fatalError?.message &&
        <ErrorBox><span>ERRO</span><span>{fatalError?.message}</span></ErrorBox>
      }

      <OpenLogs
        onClick={() => setOpenLogs(true)}
        initial={{
          y: "-100%",
          opacity: 0
        }}
        animate={{
          y: !openLogs ? 0 : "-200%",
          opacity: !openLogs ? 1 : 0
        }}
      >
        <ISvg />
      </OpenLogs>

      <LogBox
        initial={{
          y: "-100%",

        }}
        animate={{
          y: openLogs ? 0 : "-100%"
        }}
      >
        <h4>Info</h4>
        <i onClick={() => setOpenLogs(false)}><XSvg /></i>
        <LogInfoBox>
          <div>Altitude Offsseted: <b>{altitude_offset.toFixed(2)} m</b></div>
          <div>Sea Level Pressure: <b>{sea_level_pressure} hPa</b></div>
        </LogInfoBox>
        <h4>Log</h4>
        <LogList>

          {log.length > 0 &&
            log.map((log, index) => {
              return <LogItem key={index} errorLog={log.type == "Error"}>{log.message}</LogItem>
            })}
        </LogList>
      </LogBox>
      <BatteryBox >
        {
          battery < 20 ?
            <BatteryEmpty />
            : battery < 50 ?
              <BatteryHalf />
              :
              <BatteryFull />
        }
        {!isNaN(battery) && battery.toFixed(1)}
      </BatteryBox>
    </FlightDataContainer>
  );

}

export default FlightData;


function DataInfo({ info, startOrder }) {
  const [order, setOrder] = useState(startOrder)
  if (info.value == null)
    return null
  return (
    <DataInfoBox order={order}>
      <motion.b
        initial={{
          color: "#f60]"
        }}
        animate={{
          color: "#fff"
        }}
        transition={{
          duration: 1,
          ease: "easeInOut",

        }}
      >{info.name ?? ""}</motion.b>
      <motion.span
        initial={{
          color: "#09c"
        }}
        animate={{
          color: "#fff"
        }}
        transition={{
          duration: 1,
          ease: "easeInOut",

        }}
      >{typeof (info.value) == 'number' ? info.value.toFixed(2) : info.value} <small>{info.unity}</small></motion.span>
      <OrderBox>
        <OrderButton onClick={() => setOrder(p => p - 1)}><ArrowUp /></OrderButton>
        <OrderButton onClick={() => setOrder(p => p + 1)}><ArrowDown /></OrderButton>
      </OrderBox>

    </DataInfoBox>
  );
}



const BatteryBox = styled(motion.div)`
  position: absolute;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0.5rem;
  bottom: 1rem;
  right: 1.5rem;
  flex-direction: column;
  svg{
    fill: #fff;
    width: 1rem;
  }
`

const LogItem = styled.span`
  padding: 0.3rem 1rem ;
  color: ${({ errorLog }) => errorLog ? "#ff6767" : "#ffffff"};
`

const OpenLogs = styled(motion.button)`
  cursor: pointer;
  position: absolute;
  top: 1.75rem;
  right: 1rem;
  width: 2.5rem;
  height: 2.5rem;
  padding: 0.2rem;
  background-color: #fff;
  border-radius: 50%;
  border: none;
  svg{
    fill: #000;
    width: 0.5rem;
  }
`

const LogList = styled.div`
  height: 100%;
  overflow-y: auto;
  display: grid;
  grid-template-columns: 100%;
  grid-template-rows: auto;
  max-height: 50vh;
`

const LogInfoBox = styled.div`
display: grid;
grid-template-columns: 100%;
grid-template-rows: auto;
padding: 1rem;
gap: 1rem 0 ;
font-size: 1.2rem;
background-color: var(--bgColor);
  b{
    color: var(--mainColorLight);
  }
`

const LogBox = styled(motion.div)`
  padding:  1rem 0;
  background-color: var(--bgColorDark);
  font-size: 1.5rem;
  position: absolute;
  top: -0.5rem;
  width: 100%;
  left: 0;
  display: grid;
  grid-template-columns: 100%;
  grid-template-rows: auto;
  gap: 0.5rem;
  max-height: 90vh;
  min-width: 10vh;
  border-bottom: 5px  solid   var(--mainColorLight);
  
  ::-webkit-scrollbar-thumb {
    border-radius: 10px;
    background-color: var(--mainColor);
  }
  i{
    position: absolute;
    top: 1.5rem;
    color: var(--mainColorDark);

    right: 2rem;
    text-decoration: none;
    
    cursor: pointer;
    font-weight: bold;
    svg{
      width: 2rem;
      fill: var(--mainColor);
    }
  }
  h4{
    width: 100%;
    text-align: center;
    padding: 0.6rem 0.5rem;
    color: var(--mainColorLight);
    border-bottom: 1px solid  var(--mainColor);
  
  }
  span{
    &:nth-child(even){
      background-color: var(--bgColor);
    }  }
`
const ErrorBox = styled(motion.div)`
  padding: 2rem 1rem;
  background-color: var(--mainColor);
  font-size: 1.5rem;
  position: absolute;
  bottom: 0rem;
  width: 100%;
  background-color: #ff1100;
  left: 0;
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 0.5rem;
  flex-direction: column;
  z-index: 10;
  span{

    font-weight: bold;
    font-size: 2rem;
    max-width: 90%;
    text-align: center;
  }
  
`
const ApogeeInfo = styled(motion.div)`
  padding: 2rem 1rem;
  background-color: var(--mainColor);
  font-size: 1.5rem;
  position: absolute;
  top: 2rem;
  left: 0;
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 0.5rem;
`

const OrderButton = styled.button`
  width: 100%;
  height: 100%;
  font-size: 1.5rem;
  border: none;
  padding: 0.6rem 0.5rem;
  aspect-ratio: 1;
  display: flex;
  justify-content: center;
  flex-direction: column;
  cursor: pointer;
  background-color: transparent;
  svg{
    fill: #fff;
    width: 2rem;
    
  }
  `
const OrderBox = styled.div`
  height: 100%;
  display: flex;
  justify-content: space-between;
  flex-direction: column;
  gap: 0.5rem;
  margin-left: auto;
`
const DataInfoBox = styled.div`
  width: 100%;
  font-size: 1.5rem;
  font-weight: bold;
  display: grid;
  grid-template-columns: 35% 1fr 20%;
  place-items: center;
  padding: 0 1.5rem;
  background-color: var(--bgColor);
  order: ${({ order }) => order ?? 1};
  b{
    font-size: 0.9em;
    width: 100%;
    
  }
  span{
    width: 100%;
text-align: center;
    font-size: 1em;
    display: flex;
    justify-content: center;
    align-items: center;
  }
  small{
    display: flex;
    justify-content: flex-start;
    align-items: center;
    margin-left: 1rem;
    width: 3rem;
  }
`

const DataBox = styled.div`
width:  100%;
height: 100%;
display: grid;
grid-template-columns: 100%;
grid-template-rows: auto;
padding: 10% 0;
overflow-y: auto;
max-height: 90vh;
gap: 1.5rem 0;
padding-bottom: 8rem;
`

const FlightDataContainer = styled.div`
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  align-items: center;
  padding: 1rem 0;
  position: relative;
  overflow: hidden;

  
`
const Logo = styled(SvgLogo)`
  width: 60%; 
  max-width: 5rem; 
   
`
const LogoBox = styled.div`
  position: fixed;
  bottom: 0;
  display: grid;
  font-family: 'Bebas Neue', Arial;
  font-weight: bold;
  place-items: center;
  padding: 1rem;
  border-radius: 50%;
  background-image: radial-gradient(#22222299, #25252599,  transparent, transparent, transparent);
  gap: 0.3rem;
  /* background-color: var(--bgColorDark); */
  span{
    font-size: 1.2rem;
  &:nth-child(1){
    margin-left: auto;
  }
  &:nth-child(3){
    margin-right: auto;
  }

  b{
    color: var(--mainColor);  
  }
  }
  `
const Patch = styled(SvgPatch)`
  width: 30%; 
  max-width: 6rem; 
  filter: drop-shadow( 0 0 2px #dedede88);
  margin:auto;
`
const StatusBox = styled.div`
  width: 100%;
  display: flex;
  font-family: 'Bebas Neue', Arial;
  font-weight: bold;
  justify-content: space-between;
  `

const Status = styled(motion.div)`
  width: 65%;
  background-color: var(--bgColorLight);
  display: grid;
  place-items: center;

  span{
    font-family: 'Bebas Neue', Arial;
    font-size: 2.8rem;
  }
  `