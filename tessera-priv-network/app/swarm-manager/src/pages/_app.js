import '@/styles/globals.css'
import { MoralisProvider } from "react-moralis";
import "bootstrap/dist/css/bootstrap.css";


export default function App({ Component, pageProps }) {
  return (<MoralisProvider initializeOnMount={false}>
            <Component {...pageProps} />  
          </MoralisProvider>)
}
