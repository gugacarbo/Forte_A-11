import { styled } from "styled-components";
import { ReactComponent as SvgLogo } from "../../assets/logo.svg";
import { ReactComponent as SvgPatch } from "../../assets/patch.svg";
import { ReactComponent as SvgUfsc } from "../../assets/ufsc.svg";
import { ReactComponent as SvgLasc } from "../../assets/lasc.svg";
import AppContext from "../../context/data";
import { useContext } from "preact/hooks";

function Connecting() {
  const { socketStatus } = useContext(AppContext)
  return (
    <ConnectingContainer>
      <Header>
        <SvgUfsc />
        <SvgLasc />
      </Header>
      <Loader>
        <Patch searching={socketStatus != "Open"} />
        {socketStatus}
      </Loader>
      <LogoBox>
        <Logo />
        <span>
          APEX <b>ROCKETRY</b>
        </span>
      </LogoBox>
    </ConnectingContainer>
  );
}

export default Connecting;

const Header = styled.div`
  width: 100%;
  padding: 1rem;
  display: grid;
  grid-template-columns: 12% 25% 21% 25% 12%;

  svg{
    grid-column: 2/3;
    height: 100%;
    &:last-child{
      grid-column: 4/5;
    }
  }
`


const ConnectingContainer = styled.div`
  width: 100%;
  height: 100%;
  /* display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-between; */
  display: grid;
  grid-template-rows: auto 1fr auto;
  background-color: var(--bgColor);
  box-shadow: 2px 2px 5px 10px rgba(0, 0, 0, 0.5);
  position: relative;
  padding: 5% 0;
`

const LogoBox = styled.div`
  bottom: 1rem;
  width: 100%;
  display: grid;
  font-family: 'Bebas Neue', Arial;
  font-weight: bold;
  place-items: center;
  
  gap: 0.6rem;
  span{
    font-size: 1.8rem;
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
const Logo = styled(SvgLogo)`
  width: 60%; 
  max-width: 7rem; 
   
`

const Loader = styled.div`
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  font-family: 'Bebas Neue', Arial;
  font-size: 3rem;
  letter-spacing: 0.15rem;
`
const Patch = styled(SvgPatch)`
  width: 100%;
  max-width: 20rem;
  overflow: visible;
  filter: drop-shadow( 0 0 8px var(--mainColor));

  ${({ searching }) => searching && `
    animation: searchingGlow 10s infinite alternate;
    `
  }

  @keyframes searchingGlow{
    0%{
      filter: drop-shadow( 0 0 8px transparent);
    }
    10%{
      filter: drop-shadow( 0 0 8px #47caeb);
    }
    
    20%{
      filter: drop-shadow( 0 0 8px #dedede00);
    }
    30%{
      filter: drop-shadow( 0 0 8px var(--mainColor));
    }
    
    40%{
      filter: drop-shadow( 0 0 8px #dedede00);
    }
    50%{
      filter: drop-shadow( 0 0 8px #fff);
    }
    
    60%{
      filter: drop-shadow( 0 0 8px #dedede00);
    }
    70%{
      filter: drop-shadow( 0 0 8px #00ff1a86);
    }
    
    80%{
      filter: drop-shadow( 0 0 8px #dedede00);
    }
    90%{
      filter: drop-shadow( 0 0 8px #fbff0086);
    }

    100%{
      filter: drop-shadow( 0 0 8px #00000000);
    }
    
  }
`