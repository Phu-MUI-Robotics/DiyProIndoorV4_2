export default function handler(req, res) {
  if (req.method === 'GET') {
    return res.status(200).json({ message: 'API is working!', timestamp: new Date().toISOString() })
  }
  
  if (req.method === 'POST') {
    return res.status(200).json({ 
      message: 'POST received!', 
      data: req.body,
      timestamp: new Date().toISOString() 
    })
  }
  
  return res.status(405).json({ error: 'Method not allowed' })
}
